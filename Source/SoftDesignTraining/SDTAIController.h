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

    /// Starts to move directly to the target, but adjust the target point to be at least at some distance of the actor.
    /// \param target The world position where the actor will move forward.
    /// \param minimumDistance The minimum distance from the actor at which the point to move to is set.
    void SetTarget(FVector target, float minimumDistance = 200.0f);
    /// Starts to move in the direction specified to the point where a wall is detected or until the maximum distance is reached.
    /// \param headingDirection The direction to start to move.
    /// \param maximumDistance The maximum distance to move.
    void CalculateFarForwardTarget(FVector headingDirection, float maximumDistance = 10000.0f);
    /// Starts to move in the direction at which the character is looking at until a wall is detected or until the maximum distance is reached.
    void CalculateFarForwardTarget();

    /// Update the next position to go on the spline. There must be a valid spline for that.
    /// \param deltaTime Determine the amount of displacement on the spline.
    void UpdateTargetPositionOnSpline(float deltaTime);

    /// Adjust the maximum speed of the agent depending of the distance of the collision in front of the agent.
    /// \param deltaTime Used to determine the amount of speed to add or remove.
    /// \param wallCollisionDistance The distance of the next collision towards the actor.
    void SpeedControl(float deltaTime, float wallCollisionDistance);

    /// Determine the next position to go to using either the active spline or the ActiveDirectionTarget.
    /// \param deltaTime Used to determine the amount of rotation to do.
    void Move(float deltaTime);

    /// Look around the agent to find its goal. If the player is visible, the method gives the player position.
    /// If there is one or multiple visible collectibles, the method gives the closest collectible position.
    /// \param objective [out] The goal that the actor must achieve.
    /// \param target [out] A position of an object to fulfill the goal.
    void DetectObjective(ObjectiveType &objective, FVector &target);

    /// Look in front of the agent to detect a future collision. The method will only give a parallel direction to the wall
    /// the first time it detects a new wall. Also, the method must detect no collision in order to give another parallel direction on a subsequent call.
    /// \param targetDirection [out] On the first collision detection, a direction parallel to the wall, either left or right depending of the other obstacles around.
    /// \param collisionDistance [out] The distance of the forward collision.
    /// \return True if a collision is detected, false otherwise.
    bool DetectWalls(FVector &targetDirection, float &collisionDistance);
    bool DetectWalls(FVector& targetDirection, float& collisionDistance, FVector hitDirection, float hitDist);
    /// Reset the wall detection even if there was a collision detected.
    void ResetWallsDetection();

    /// The normal of the last wall detected during a call to DetectWalls.
    FVector LastImpactNormal = FVector::ZeroVector;

    /// When the first wall is detected, indicates the rotation direction axis.
    int RotationDirection = 1;

    /// The actual objective. When changing the objective, some variables need to be reset.
    ObjectiveType CurrentObjective = ObjectiveType::WALKING;

    /// The current direction objective when no spline is defined.
    /// If no active direction and no spline, the agent moves forward.
    FVector ActiveDirectionTarget = FVector::ZeroVector;

    /// The last position passed to MoveToLocation.
    FVector TargetMoveTo;

    /// The spline that the agent follows, depending of the value of SplineDistance.
    USplineComponent* SplineChassing;
    /// When different than -1, the agent is assumed to be on the spline and this value represents its progression on the spline.
    float SplineDistance = -1.0f;
};
