// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/SplineComponent.h"
#include "AIController.h"
#include "CoreMinimal.h"

#include "SDTAIController.generated.h"

/**
 *
 */
UCLASS(ClassGroup = AI, config = Game)
class SOFTDESIGNTRAINING_API ASDTAIController : public AAIController
{
    GENERATED_BODY()
public:
    virtual void BeginPlay() override;

    virtual void OnMoveCompleted(FAIRequestID RequestID, EPathFollowingResult::Type Result) override;

    virtual void Tick(float deltaTime) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxSpeed = 400.0f;

    float VisionDistance = 800.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ClampMin = 0.0, ClampMax = 180.0))
    double VisionAngle = 60.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Acceleration = 250.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float ForwardWallRayCastDist = 150.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float SidesWallRayCastDist = 300.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float RotationAngleBySecond = 200;

private:
    enum class ObjectiveType
    {
        CHASSING,
        FLEEING,
        WALKING
    };

    void CalculateFarForwardTarget(FVector headingTarget);
    void CalculateFarForwardTarget();
    void MoveTowardsTarget(FVector target);
    void DetectObjective(ObjectiveType& objective, FVector& target);
    //bool DetectCollectible(FVector& targetDirection);
    bool DetectWalls(FVector& targetDirection, float& collisionDistance);
    void ResetWallsDetection();
    void SpeedControl(float deltaTime, float wallCollisionDistance);
    //void Move(float deltaTime, FVector targetDirection);
    void Move(float deltaTime);
    //bool IsInVisionCone(UWorld* world, AActor* pawn, AActor* targetActor);

    USplineComponent *chassingSpline;
    float SplineDistance = -1.0f;
    FVector lastImpactNormal = FVector::ZeroVector;
    FVector lastTargetDirectionForWalls = FVector::ZeroVector;
    int rotationDirection = 1;

    FVector previousRotationAxis = FVector::ZeroVector;
    FVector targetMoveTo;

    FVector ActiveDirectionTarget = FVector::ZeroVector;

    float TargetSpeed = 0.0f;
};
