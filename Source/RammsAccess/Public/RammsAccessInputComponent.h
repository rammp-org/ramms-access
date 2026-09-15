// Copyright (c) RAMMP. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RammsAccessInputComponent.generated.h"

class FSocket;
class UGripperControllerComponent;
class UKinovaGen3ControllerComponent;
class URammsDifferentialDriveController;

/** One parsed intent packet (protocol v1 — see doc/PLAN.md). */
USTRUCT(BlueprintType)
struct RAMMSACCESS_API FRammsAccessIntent
{
	GENERATED_BODY()

	/** Source adapter name (e.g. "galea", "pointer", "demo"). */
	UPROPERTY(BlueprintReadOnly, Category = "Ramms|Access")
	FString Source;

	/** Wheelchair-joystick drive input: X = turn, Y = forward, -1..1. */
	UPROPERTY(BlueprintReadOnly, Category = "Ramms|Access")
	FVector2D Drive = FVector2D::ZeroVector;

	/** Normalized end-effector linear rates (EE-local xyz), -1..1. */
	UPROPERTY(BlueprintReadOnly, Category = "Ramms|Access")
	FVector EELinear = FVector::ZeroVector;

	/** Normalized end-effector angular rates (pitch/yaw/roll), -1..1. */
	UPROPERTY(BlueprintReadOnly, Category = "Ramms|Access")
	FRotator EEAngular = FRotator::ZeroRotator;

	/** Decoder confidence 0..1 — scales the continuous rates. */
	UPROPERTY(BlueprintReadOnly, Category = "Ramms|Access")
	float Confidence = 1.0f;

	/** Sender sequence number (staleness rejection — valid within one session). */
	UPROPERTY(BlueprintReadOnly, Category = "Ramms|Access")
	int64 Sequence = -1;

	/** Sender session id: random per publisher instance. A changed sid resets
	 *  the staleness cursor (a restarted adapter starts again at seq 1). */
	UPROPERTY(BlueprintReadOnly, Category = "Ramms|Access")
	int64 SessionId = -1;

	bool bHasDrive = false;
	bool bHasEE = false;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnAccessIntentReceived, const FRammsAccessIntent&, Intent);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnAccessEStop);
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnAccessWatchdogTimeout);

/**
 * Consumes the ramms-access intent stream (UDP JSON, latest-wins) and drives
 * the robot through its control surface (IRammsControlSink, the sibling
 * URammsRobotControlSurfaceComponent) with Source = Autonomy: drive intents
 * -> drive.forward / drive.turn, end-effector intents -> arm.forward /
 * strafe / up / pitch / yaw / roll rate axes, gripper and sync events ->
 * gripper.* / arm.resync actions. Autonomy outranks local input in the
 * surface's arbitration for the surface's hold window, the way the old
 * external-drive path did. Add to the robot pawn; the external
 * "ramms-access" hub (python/) does device handling and decoding and
 * publishes intents.
 *
 * Without a control surface on the pawn (bUseControlSurface off, or none
 * present) it falls back to the legacy direct path: SetExternalDriveInput,
 * ApplyEndEffectorTeleopInput and the gripper controller by name.
 *
 * Safety: a watchdog zeroes drive and EE rates if no valid packet arrives
 * within WatchdogTimeoutMs (the arm holds pose; the base stops). An "estop"
 * event latches everything zero until "resume".
 */
UCLASS(ClassGroup = (Ramms), meta = (BlueprintSpawnableComponent))
class RAMMSACCESS_API URammsAccessInputComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URammsAccessInputComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** UDP port the intent stream arrives on. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Access|Network")
	int32 Port = 30040;

	/** Open the socket on BeginPlay. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Access|Network")
	bool bListenOnBeginPlay = true;

	/** Zero all continuous control if no valid packet arrives within this window. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Access|Safety", meta = (ClampMin = "50"))
	float WatchdogTimeoutMs = 250.0f;

	/** End-effector linear speed at full deflection (cm/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Access|Mapping", meta = (ClampMin = "0.1"))
	float EELinearSpeedCmPerSecond = 15.0f;

	/** End-effector angular speed at full deflection (deg/s). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Access|Mapping", meta = (ClampMin = "0.1"))
	float EEAngularSpeedDegPerSecond = 30.0f;

	/** Apply EE linear input in the end-effector local frame (matches teleop). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Access|Mapping")
	bool bEEInputInLocalFrame = true;

	/** Scale continuous rates by the packet's decoder confidence. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Access|Mapping")
	bool bScaleByConfidence = true;

	/** Route through the pawn's control surface (IRammsControlSink) when one is
	 *  present. EE rates then move at the arm teleop component's speeds, not
	 *  EELinearSpeedCmPerSecond / EEAngularSpeedDegPerSecond. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Access|Mapping")
	bool bUseControlSurface = true;

	/** Optional component name overrides when the owner has multiples (legacy path). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Access|Mapping")
	FName DriveControllerName = NAME_None;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Access|Mapping")
	FName KinovaControllerName = NAME_None;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Access|Mapping")
	FName GripperControllerName = NAME_None;

	/** Fired for every applied intent packet (UI/telemetry). */
	UPROPERTY(BlueprintAssignable, Category = "Ramms|Access|Events")
	FOnAccessIntentReceived OnIntentReceived;

	/** Fired when an estop event latches control off. */
	UPROPERTY(BlueprintAssignable, Category = "Ramms|Access|Events")
	FOnAccessEStop OnEStop;

	/** Fired once each time the watchdog zeroes control. */
	UPROPERTY(BlueprintAssignable, Category = "Ramms|Access|Events")
	FOnAccessWatchdogTimeout OnWatchdogTimeout;

	UFUNCTION(BlueprintCallable, Category = "Ramms|Access")
	bool StartListening();

	UFUNCTION(BlueprintCallable, Category = "Ramms|Access")
	void StopListening();

	UFUNCTION(BlueprintPure, Category = "Ramms|Access")
	bool IsEStopLatched() const { return bEStopLatched; }

	/** Clear the estop latch. Also cleared by a "resume" event from the hub —
	 *  every adapter sends one on startup, so starting a fresh adapter session
	 *  resumes control. CallInEditor: shows as a button on the component's
	 *  details panel during PIE. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "Ramms|Access")
	void ClearEStop() { bEStopLatched = false; }

private:
	void ResolveTargets();
	bool ParseIntent(const FString& Json, FRammsAccessIntent& Out, TArray<FString>& OutEvents) const;
	void ApplyIntent(const FRammsAccessIntent& Intent, float DeltaTime);
	void HandleEvent(const FString& Event);
	void ZeroControl();

	// Control-surface path (IRammsControlSink on the owner).
	bool SinkAvailable() const;
	void SinkSet(FName Id, float Value);
	void SinkRelease(FName Id);
	bool SinkTrigger(FName Id);

	FSocket* Socket = nullptr;

	/** The owner's control surface (implements IRammsControlSink); null = legacy path. */
	UPROPERTY(Transient)
	TObjectPtr<UObject> ControlSink;

	UPROPERTY(Transient)
	TObjectPtr<URammsDifferentialDriveController> DriveController;
	UPROPERTY(Transient)
	TObjectPtr<UKinovaGen3ControllerComponent> KinovaController;
	UPROPERTY(Transient)
	TObjectPtr<UGripperControllerComponent> GripperController;

	double LastValidPacketSeconds = -1.0;
	int64  LastSequence = -1;
	int64  LastSessionId = -1;
	bool   bEStopLatched = false;
	bool   bWatchdogTripped = false;
	/** True once the current suppression episode (watchdog/estop) has zeroed
	 *  control — the zero must happen exactly once per episode, not per frame. */
	bool   bControlZeroed = false;

	/** Latest continuous intent, re-applied each tick until superseded or timed out. */
	FRammsAccessIntent LatestIntent;
	bool bHaveIntent = false;
};
