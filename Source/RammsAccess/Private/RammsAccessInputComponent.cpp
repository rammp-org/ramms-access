// Copyright (c) RAMMP. All rights reserved.

#include "RammsAccessInputComponent.h"

#include "Common/UdpSocketBuilder.h"
#include "Dom/JsonObject.h"
#include "GameFramework/Actor.h"
#include "GripperControllerComponent.h"
#include "KinovaGen3ControllerComponent.h"
#include "RammsControlIds.h"
#include "RammsControlSink.h"
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

	ControlSink = nullptr;
	if (bUseControlSurface)
	{
		TArray<UActorComponent*> All;
		Owner->GetComponents(All);
		for (UActorComponent* C : All)
		{
			if (C && C->GetClass()->ImplementsInterface(URammsControlSink::StaticClass()))
			{
				ControlSink = C;
				break;
			}
		}
	}

	UE_LOG(LogRammsAccess, Log, TEXT("[%s] targets: surface=%s arm=%s gripper=%s"), *GetPathName(),
		*GetNameSafe(ControlSink), *GetNameSafe(KinovaController), *GetNameSafe(GripperController));
}

// --- control-surface path -------------------------------------------------------

namespace
{
	const FName ArmForwardId(TEXT("arm.forward"));
	const FName ArmStrafeId(TEXT("arm.strafe"));
	const FName ArmUpId(TEXT("arm.up"));
	const FName ArmPitchId(TEXT("arm.pitch"));
	const FName ArmYawId(TEXT("arm.yaw"));
	const FName ArmRollId(TEXT("arm.roll"));
	const FName ArmResyncId(TEXT("arm.resync"));
	const FName GripperOpenId(TEXT("gripper.open"));
	const FName GripperCloseId(TEXT("gripper.close"));
	const FName GripperToggleId(TEXT("gripper.toggle"));
} // namespace

bool URammsAccessInputComponent::SinkAvailable() const
{
	return ControlSink != nullptr;
}

void URammsAccessInputComponent::SinkSet(FName Id, float Value)
{
	if (ControlSink)
	{
		IRammsControlSink::Execute_SetAxis(ControlSink, Id, Value, ERammsControlSource::Autonomy);
	}
}

void URammsAccessInputComponent::SinkRelease(FName Id)
{
	if (ControlSink)
	{
		IRammsControlSink::Execute_ReleaseAxis(ControlSink, Id, ERammsControlSource::Autonomy);
	}
}

bool URammsAccessInputComponent::SinkTrigger(FName Id)
{
	return ControlSink && IRammsControlSink::Execute_TriggerAction(ControlSink, Id, ERammsControlSource::Autonomy);
}

bool URammsAccessInputComponent::ParseIntent(const FString& Json, FRammsAccessIntent& Out, TArray<FString>& OutEvents) const
{
	TSharedPtr<FJsonObject>			Root;
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
	else if (Event == TEXT("gripper_open"))
	{
		if (SinkAvailable())
		{
			SinkTrigger(GripperOpenId); // a refusal is the surface's decision (unknown / arbitration), not a reason to bypass it
		}
		else if (GripperController != nullptr)
		{
			GripperController->Open();
		}
	}
	else if (Event == TEXT("gripper_close"))
	{
		if (SinkAvailable())
		{
			SinkTrigger(GripperCloseId); // a refusal is the surface's decision (unknown / arbitration), not a reason to bypass it
		}
		else if (GripperController != nullptr)
		{
			GripperController->Close();
		}
	}
	else if (Event == TEXT("gripper_toggle"))
	{
		if (SinkAvailable())
		{
			SinkTrigger(GripperToggleId); // a refusal is the surface's decision (unknown / arbitration), not a reason to bypass it
		}
		else if (GripperController != nullptr)
		{
			GripperController->Toggle();
		}
	}
	else if (Event == TEXT("sync_target"))
	{
		if (SinkAvailable())
		{
			SinkTrigger(ArmResyncId);
		}
		else if (KinovaController != nullptr)
		{
			KinovaController->SnapEndEffectorTargetToCurrentPose();
		}
	}
	else
	{
		UE_LOG(LogRammsAccess, Verbose, TEXT("[%s] unhandled event '%s'"), *GetPathName(), *Event);
	}
}

void URammsAccessInputComponent::ZeroControl()
{
	if (SinkAvailable())
	{
		// Let go of what this component drove — and only that: Autonomy
		// outranks local sources in the sink, so releasing an axis it never
		// touched would cancel another controller's input. The drive springs
		// back, the arm holds its target, and the hold lapses so local input
		// resumes.
		if (bSinkDriveActive)
		{
			SinkRelease(RammsControlIds::Drive::Forward());
			SinkRelease(RammsControlIds::Drive::Turn());
			bSinkDriveActive = false;
		}
		if (bSinkEEActive)
		{
			for (const FName& Id : { ArmForwardId, ArmStrafeId, ArmUpId, ArmPitchId, ArmYawId, ArmRollId })
			{
				SinkRelease(Id);
			}
			bSinkEEActive = false;
		}
		bHaveIntent = false;
		return;
	}
	// The arm needs no explicit zero: EE motion only happens while we actively
	// feed teleop input; withholding input holds the current target pose.
	bHaveIntent = false;
}

void URammsAccessInputComponent::ApplyIntent(const FRammsAccessIntent& Intent, float DeltaTime)
{
	const float Scale = bScaleByConfidence ? Intent.Confidence : 1.0f;

	if (SinkAvailable())
	{
		// Autonomy outranks local input in the surface's arbitration; the arm
		// contributor integrates the rate axes at its own teleop speeds.
		// Packets are latest-wins per domain: when the newest packet omits a
		// domain an earlier one drove, release that domain once (not every
		// tick, which would fight whoever drives it next).
		if (Intent.bHasDrive)
		{
			SinkSet(RammsControlIds::Drive::Forward(), static_cast<float>(Intent.Drive.Y) * Scale);
			SinkSet(RammsControlIds::Drive::Turn(), static_cast<float>(Intent.Drive.X) * Scale);
			bSinkDriveActive = true;
		}
		else if (bSinkDriveActive)
		{
			SinkRelease(RammsControlIds::Drive::Forward());
			SinkRelease(RammsControlIds::Drive::Turn());
			bSinkDriveActive = false;
		}
		if (Intent.bHasEE)
		{
			SinkSet(ArmForwardId, static_cast<float>(Intent.EELinear.X) * Scale);
			SinkSet(ArmStrafeId, static_cast<float>(Intent.EELinear.Y) * Scale);
			SinkSet(ArmUpId, static_cast<float>(Intent.EELinear.Z) * Scale);
			SinkSet(ArmPitchId, static_cast<float>(Intent.EEAngular.Pitch) * Scale);
			SinkSet(ArmYawId, static_cast<float>(Intent.EEAngular.Yaw) * Scale);
			SinkSet(ArmRollId, static_cast<float>(Intent.EEAngular.Roll) * Scale);
			bSinkEEActive = true;
		}
		else if (bSinkEEActive)
		{
			for (const FName& Id : { ArmForwardId, ArmStrafeId, ArmUpId, ArmPitchId, ArmYawId, ArmRollId })
			{
				SinkRelease(Id);
			}
			bSinkEEActive = false;
		}
		return;
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
		uint8					  Buffer[8192];
		int32					  BytesRead = 0;
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
