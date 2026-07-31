// Copyright (c) RAMMP. All rights reserved.

#include "RammsAccessInputComponent.h"

#include "Common/UdpSocketBuilder.h"
#include "Dom/JsonObject.h"
#include "GameFramework/Actor.h"
#include "GripperControllerComponent.h"
#include "KinovaGen3ControllerComponent.h"
#include "RammsDifferentialDriveController.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Sockets.h"
#include "SocketSubsystem.h"

DEFINE_LOG_CATEGORY_STATIC(LogRammsAccess, Log, All);

URammsAccessInputComponent::URammsAccessInputComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void URammsAccessInputComponent::BeginPlay()
{
	Super::BeginPlay();
	ResolveTargets();
	if (bListenOnBeginPlay)
	{
		StartListening();
	}
}

void URammsAccessInputComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopListening();
	Super::EndPlay(EndPlayReason);
}

bool URammsAccessInputComponent::StartListening()
{
	if (Socket != nullptr)
	{
		return true;
	}
	// Deliberately NOT reusable: a control port must have exactly one consumer.
	// With address reuse, a duplicate component instance binds silently and both
	// receive the stream — a stale/latched duplicate then fights the live one
	// (observed as per-frame zero writes clobbering applied intents). A loud
	// bind failure surfaces the duplicate instead.
	Socket = FUdpSocketBuilder(TEXT("RammsAccessIntent"))
				 .AsNonBlocking()
				 .BoundToAddress(FIPv4Address::Any)
				 .BoundToPort(Port)
				 .WithReceiveBufferSize(64 * 1024);
	if (Socket == nullptr)
	{
		UE_LOG(LogRammsAccess, Error, TEXT("[%s] failed to bind UDP port %d — intent stream disabled"), *GetPathName(), Port);
		return false;
	}
	UE_LOG(LogRammsAccess, Log, TEXT("[%s] listening for access intents on UDP %d"), *GetPathName(), Port);
	return true;
}

void URammsAccessInputComponent::StopListening()
{
	if (Socket != nullptr)
	{
		ISocketSubsystem* Subsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
		Socket->Close();
		if (Subsystem != nullptr)
		{
			Subsystem->DestroySocket(Socket);
		}
		Socket = nullptr;
	}
}

void URammsAccessInputComponent::ResolveTargets()
{
	AActor* Owner = GetOwner();
	if (Owner == nullptr)
	{
		return;
	}

	TArray<URammsDifferentialDriveController*> Drives;
	Owner->GetComponents(Drives);
	for (URammsDifferentialDriveController* C : Drives)
	{
		if (DriveControllerName == NAME_None || C->GetFName() == DriveControllerName)
		{
			DriveController = C;
			break;
		}
	}

	TArray<UKinovaGen3ControllerComponent*> Arms;
	Owner->GetComponents(Arms);
	for (UKinovaGen3ControllerComponent* C : Arms)
	{
		if (KinovaControllerName == NAME_None || C->GetFName() == KinovaControllerName)
		{
			KinovaController = C;
			break;
		}
	}

	TArray<UGripperControllerComponent*> Grippers;
	Owner->GetComponents(Grippers);
	for (UGripperControllerComponent* C : Grippers)
	{
		if (GripperControllerName == NAME_None || C->GetFName() == GripperControllerName)
		{
			GripperController = C;
			break;
		}
	}

	UE_LOG(LogRammsAccess, Log, TEXT("[%s] targets: drive=%s arm=%s gripper=%s"), *GetPathName(),
		*GetNameSafe(DriveController), *GetNameSafe(KinovaController), *GetNameSafe(GripperController));
}

bool URammsAccessInputComponent::ParseIntent(const FString& Json, FRammsAccessIntent& Out, TArray<FString>& OutEvents) const
{
	TSharedPtr<FJsonObject> Root;
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
	if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
	{
		return false;
	}
	int32 Version = 0;
	if (!Root->TryGetNumberField(TEXT("v"), Version) || Version != 1)
	{
		return false;
	}

	Root->TryGetStringField(TEXT("src"), Out.Source);
	double Seq = -1.0;
	if (Root->TryGetNumberField(TEXT("seq"), Seq))
	{
		Out.Sequence = static_cast<int64>(Seq);
	}
	double Sid = -1.0;
	if (Root->TryGetNumberField(TEXT("sid"), Sid))
	{
		Out.SessionId = static_cast<int64>(Sid);
	}
	double Conf = 1.0;
	if (Root->TryGetNumberField(TEXT("conf"), Conf))
	{
		Out.Confidence = FMath::Clamp(static_cast<float>(Conf), 0.0f, 1.0f);
	}

	const TSharedPtr<FJsonObject>* DriveObj = nullptr;
	if (Root->TryGetObjectField(TEXT("drive"), DriveObj) && DriveObj != nullptr && DriveObj->IsValid())
	{
		Out.Drive.X = FMath::Clamp((*DriveObj)->GetNumberField(TEXT("x")), -1.0, 1.0);
		Out.Drive.Y = FMath::Clamp((*DriveObj)->GetNumberField(TEXT("y")), -1.0, 1.0);
		Out.bHasDrive = true;
	}

	const TSharedPtr<FJsonObject>* EEObj = nullptr;
	if (Root->TryGetObjectField(TEXT("ee"), EEObj) && EEObj != nullptr && EEObj->IsValid())
	{
		const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
		if ((*EEObj)->TryGetArrayField(TEXT("lin"), Arr) && Arr != nullptr && Arr->Num() >= 3)
		{
			Out.EELinear = FVector(
				FMath::Clamp((*Arr)[0]->AsNumber(), -1.0, 1.0),
				FMath::Clamp((*Arr)[1]->AsNumber(), -1.0, 1.0),
				FMath::Clamp((*Arr)[2]->AsNumber(), -1.0, 1.0));
			Out.bHasEE = true;
		}
		if ((*EEObj)->TryGetArrayField(TEXT("ang"), Arr) && Arr != nullptr && Arr->Num() >= 3)
		{
			Out.EEAngular = FRotator(
				FMath::Clamp((*Arr)[0]->AsNumber(), -1.0, 1.0),
				FMath::Clamp((*Arr)[1]->AsNumber(), -1.0, 1.0),
				FMath::Clamp((*Arr)[2]->AsNumber(), -1.0, 1.0));
			Out.bHasEE = true;
		}
	}

	const TArray<TSharedPtr<FJsonValue>>* Events = nullptr;
	if (Root->TryGetArrayField(TEXT("events"), Events) && Events != nullptr)
	{
		for (const TSharedPtr<FJsonValue>& V : *Events)
		{
			FString Event;
			if (V->TryGetString(Event))
			{
				OutEvents.Add(MoveTemp(Event));
			}
		}
	}
	return true;
}

void URammsAccessInputComponent::HandleEvent(const FString& Event)
{
	if (Event == TEXT("estop"))
	{
		bEStopLatched = true;
		ZeroControl();
		bControlZeroed = true;
		OnEStop.Broadcast();
		UE_LOG(LogRammsAccess, Warning, TEXT("[%s] ESTOP latched by intent stream (clear via 'resume' event or ClearEStop)"), *GetPathName());
		return;
	}
	if (Event == TEXT("resume"))
	{
		if (bEStopLatched)
		{
			UE_LOG(LogRammsAccess, Log, TEXT("[%s] estop cleared by 'resume' event"), *GetPathName());
		}
		bEStopLatched = false;
		return;
	}
	if (bEStopLatched)
	{
		return; // only estop/resume are honored while latched
	}
	if (Event == TEXT("stop"))
	{
		ZeroControl();
	}
	else if (Event == TEXT("gripper_open") && GripperController != nullptr)
	{
		GripperController->Open();
	}
	else if (Event == TEXT("gripper_close") && GripperController != nullptr)
	{
		GripperController->Close();
	}
	else if (Event == TEXT("gripper_toggle") && GripperController != nullptr)
	{
		GripperController->Toggle();
	}
	else if (Event == TEXT("sync_target") && KinovaController != nullptr)
	{
		KinovaController->SnapEndEffectorTargetToCurrentPose();
	}
	else
	{
		UE_LOG(LogRammsAccess, Verbose, TEXT("[%s] unhandled event '%s'"), *GetPathName(), *Event);
	}
}

void URammsAccessInputComponent::ZeroControl()
{
	if (DriveController != nullptr)
	{
		// External path: zeroes the drive AND briefly extends external priority,
		// so the stop lands even though the pawn's per-tick input writes continue;
		// the hold then expires and the player joystick resumes automatically.
		DriveController->SetExternalDriveInput(FVector2D::ZeroVector);
	}
	// The arm needs no explicit zero: EE motion only happens while we actively
	// feed teleop input; withholding input holds the current target pose.
	bHaveIntent = false;
}

void URammsAccessInputComponent::ApplyIntent(const FRammsAccessIntent& Intent, float DeltaTime)
{
	const float Scale = bScaleByConfidence ? Intent.Confidence : 1.0f;

	if (Intent.bHasDrive && DriveController != nullptr)
	{
		// External path wins arbitration against the pawn's per-tick joystick
		// writes (see URammsDifferentialDriveController::SetExternalDriveInput).
		DriveController->SetExternalDriveInput(Intent.Drive * Scale);
	}
	if (Intent.bHasEE && KinovaController != nullptr && (!Intent.EELinear.IsNearlyZero() || !Intent.EEAngular.IsNearlyZero()))
	{
		KinovaController->ApplyEndEffectorTeleopInput(
			Intent.EELinear * Scale,
			FRotator(Intent.EEAngular.Pitch * Scale, Intent.EEAngular.Yaw * Scale, Intent.EEAngular.Roll * Scale),
			DeltaTime,
			bEEInputInLocalFrame,
			EELinearSpeedCmPerSecond,
			EEAngularSpeedDegPerSecond);
	}
}

void URammsAccessInputComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (Socket != nullptr)
	{
		// Drain every pending datagram; continuous fields are latest-wins,
		// discrete events are processed in arrival order.
		uint8	 Buffer[8192];
		int32	 BytesRead = 0;
		TSharedRef<FInternetAddr> Sender = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
		while (Socket->RecvFrom(Buffer, sizeof(Buffer) - 1, BytesRead, *Sender))
		{
			if (BytesRead <= 0)
			{
				break;
			}
			Buffer[BytesRead] = 0;
			const FString Json = FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buffer)));

			FRammsAccessIntent Intent;
			TArray<FString>	   Events;
			if (!ParseIntent(Json, Intent, Events))
			{
				continue;
			}
			// A new sender session resets the staleness cursor — a restarted
			// adapter starts counting from 1 again, and without this every
			// packet of the new session (including its 'resume') would be
			// rejected as stale.
			if (Intent.SessionId >= 0 && Intent.SessionId != LastSessionId)
			{
				UE_LOG(LogRammsAccess, Log, TEXT("[%s] new intent session %lld (src '%s')"), *GetPathName(), Intent.SessionId, *Intent.Source);
				LastSessionId = Intent.SessionId;
				LastSequence = -1;
			}
			// Reject stale/reordered packets within the same session.
			if (Intent.Sequence >= 0 && LastSequence >= 0 && Intent.Sequence <= LastSequence)
			{
				continue;
			}
			if (Intent.Sequence >= 0)
			{
				LastSequence = Intent.Sequence;
			}

			LastValidPacketSeconds = FPlatformTime::Seconds();
			bWatchdogTripped = false;

			for (const FString& Event : Events)
			{
				HandleEvent(Event);
			}
			if (Intent.bHasDrive || Intent.bHasEE)
			{
				LatestIntent = Intent;
				bHaveIntent = true;
			}
			OnIntentReceived.Broadcast(Intent);
		}
	}

	// Watchdog / estop suppression. Zero ONCE per suppression episode: a
	// suppressed component must not keep writing zeros every frame — that
	// fights other legitimate writers (player input, or another consumer)
	// and turns a latched instance into a control-stream jammer.
	const bool bTimedOut = LastValidPacketSeconds < 0.0
		|| (FPlatformTime::Seconds() - LastValidPacketSeconds) * 1000.0 > WatchdogTimeoutMs;
	if (bTimedOut || bEStopLatched)
	{
		if (!bControlZeroed)
		{
			ZeroControl();
			bControlZeroed = true;
			if (bTimedOut && !bWatchdogTripped && LastValidPacketSeconds >= 0.0)
			{
				bWatchdogTripped = true;
				OnWatchdogTimeout.Broadcast();
				UE_LOG(LogRammsAccess, Warning, TEXT("[%s] watchdog: intent stream silent > %.0f ms — control zeroed"), *GetPathName(), WatchdogTimeoutMs);
			}
		}
		return;
	}
	bControlZeroed = false;

	if (bHaveIntent)
	{
		ApplyIntent(LatestIntent, DeltaTime);
	}
}
