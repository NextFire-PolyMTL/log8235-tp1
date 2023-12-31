// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTAIController.h"
#include "SoftDesignTraining.h"
#include "SDTUtils.h"
#include "SDTCollectible.h"

const auto collisionDurationDebug = 5.0f;

FColor ColorIfHit(bool hit)
{
    return hit ? FColor::Red : FColor::Green;
}

float ThicknessIfHit(bool hit)
{
    return hit ? 5.0f : 0.0f;
}

FString PathFollowingResultToString(EPathFollowingResult::Type result)
{
    switch (result)
    {
    case EPathFollowingResult::Aborted:
        return "Aborted";

    case EPathFollowingResult::Blocked:
        return "Blocked";

    case EPathFollowingResult::Invalid:
        return "Invalid";

    case EPathFollowingResult::OffPath:
        return "OffPath";

    case EPathFollowingResult::Success:
        return "Success";

    default:
        return FString::FromInt(result);
    }
}

int ASDTAIController::IntRotationDirection() const
{
    return static_cast<int>(RotationDirection);
}

void ASDTAIController::SetTarget(FVector target, float minimumDistance)
{
    auto actorLocation = GetCharacter()->GetActorLocation();
    auto movementActorTarget = target - actorLocation;
    auto movementSize = movementActorTarget.Size();

    // Extend the target to be at a distance of at least minimumDistance from the actor.
    // Having a minimum distance prevents the call to MoveToLocation to complete immediately with Success. When there is a Success, the
    // velocity resets to 0 immediately and we want to avoid that.
    TargetMoveTo = movementSize < minimumDistance ? actorLocation + movementActorTarget * (minimumDistance / movementSize) : target;

    MoveToLocation(TargetMoveTo, -1, true, false, false);
}

void ASDTAIController::CalculateFarForwardTarget(FVector headingDirection, float maximumDistance)
{
    auto actorLocation = GetCharacter()->GetActorLocation();
    auto target = actorLocation + maximumDistance * headingDirection;

    FHitResult hitResult;
    if (SDTUtils::BlockingRayAgent(GetWorld(), actorLocation, target, hitResult))
    {
        target = GetCharacter()->GetActorLocation() + hitResult.Distance * headingDirection;
    }
    SetTarget(target);
}

void ASDTAIController::CalculateFarForwardTarget()
{
    CalculateFarForwardTarget(GetCharacter()->GetActorForwardVector());
}


void ASDTAIController::BeginPlay()
{
    Super::BeginPlay();

    // This spline is used when there is an obstacle that needs to be avoided. It is created dynamically because the AIController logic
    // absolutely needs a USplineComponent to work. If we would get such a component from the Character, we don't know if there is an existing spline
    // component on the character. If the USplineComponent is created on the AIController itself, it seems to be deleted after a while and the program crashes.
    // Also, if not created on the character, the debug draw of the spline does not work well.
    SplineChassing = Cast<USplineComponent>(GetCharacter()->AddComponentByClass(USplineComponent::StaticClass(), false, FTransform::Identity, false));
    SplineChassing->ClearSplinePoints();
    SplineChassing->SetDrawDebug(true);
    SplineChassing->SetAbsolute(true, true, true);

    CalculateFarForwardTarget();
}

void ASDTAIController::OnMoveCompleted(FAIRequestID requestID, EPathFollowingResult::Type result)
{
    if (result != EPathFollowingResult::Aborted)
    {
        // That should not happen since we always set a new location before the actor reaches the destination specified in MoveToLocation.
        // However, IF it happens, we will restart a movement of the actor in straight line.
        // Note there is a recursion danger if inside OnMoveCompleted we call MoveToLocation with a point where the actor is already at
        // because OnMoveCompleted will be called again immediately with EPathFollowingResult::Success.
        GEngine->AddOnScreenDebugMessage(INDEX_NONE, 3.0f, FColor::Black, FString("Path Following Result: ") + PathFollowingResultToString(result));
        CalculateFarForwardTarget();
    }
}

void ASDTAIController::Tick(float deltaTime)
{
    Super::Tick(deltaTime);
    auto hasForwardHit = false;
    FHitResult forwardHit;
    auto target = FVector::ZeroVector;
    auto parallelObstacleDirection = FVector::ZeroVector;
    ObjectiveType objective;

    DetectObjective(objective, target);
    // If the objective changed, reset some variables depending of the state we were in.
    if (objective != CurrentObjective)
    {
        switch (CurrentObjective)
        {
        case ObjectiveType::CHASSING:
            SplineChassing->ClearSplinePoints(true);
            SplineDistance = -1.0f;
            ActiveDirectionTarget = GetCharacter()->GetActorForwardVector();
            break;

        case ObjectiveType::WALKING:
            ResetObstaclesDetection();
            break;

        case ObjectiveType::FLEEING:
            ResetObstaclesDetection();
            ActiveDirectionTarget = GetCharacter()->GetActorForwardVector();
            break;
        }

        CurrentObjective = objective;
    }

    switch (objective)
    {
    case ObjectiveType::CHASSING:
    {
        // Check if we already calculated a spline.
        if (SplineDistance != -1.0f)
        {
            FHitResult hitData;
            // Check if we have a direct path to the target. If yes, stop following the spline.
            if (!SDTUtils::SweepOverlapAgent(GetWorld(), GetCharacter()->GetActorLocation(), target, GetCharacter()->GetCapsuleComponent()->GetCollisionShape(), hitData))
            {
                DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Chassing direct", nullptr, FColor::Green, 0.0f, true);
                SplineDistance = -1.0f;
                ActiveDirectionTarget = target - GetCharacter()->GetActorLocation();
            }
            else
            {
                // Follow the spline. We assume that the spline does not hit anything.
                DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Chassing spline", nullptr, FColor::Green, 0.0f, true);
                UpdateTargetPositionOnSpline(deltaTime);
            }
        }
        else
        {
            TArray<FVector> pathPoints;
            target.Z = GetCharacter()->GetActorLocation().Z;
            // Find a path to the target.
            if (SDTUtils::FindPathToLocation(GetWorld(), GetCharacter()->GetActorLocation(), target, GetCharacter()->GetActorUpVector(), GetCharacter()->GetCapsuleComponent(), pathPoints))
            {
                if (pathPoints.Num() <= 2)
                {
                    // We can go directly onto the target.
                    DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Chassing direct", nullptr, FColor::Green, 0.0f, true);
                    ActiveDirectionTarget = target - GetCharacter()->GetActorLocation();
                }
                else
                {
                    // Create a spline to reach the target.
                    DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Chassing spline", nullptr, FColor::Green, 0.0f, true);
                    SplineChassing->ClearSplinePoints(false);
                    SplineChassing->SetSplinePoints(pathPoints, ESplineCoordinateSpace::World, true);
                    auto totalLength = SplineChassing->GetSplineLength();
                    for (auto length = 0; length <= totalLength; length = length + totalLength / 10)
                    {
                        DrawDebugPoint(GetWorld(), SplineChassing->GetWorldLocationAtDistanceAlongSpline(length), 5.0f, FColor::Blue, false, 5.0f);
                    }
                    SplineDistance = GetCharacter()->GetCapsuleComponent()->GetScaledCapsuleRadius();
                    UpdateTargetPositionOnSpline(deltaTime);
                }
            }
            else
            {
                DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Chassing no path", nullptr, FColor::Green, 0.0f, true);
                hasForwardHit = AvoidObstacles(parallelObstacleDirection, forwardHit, target);
                if (parallelObstacleDirection != FVector::ZeroVector)
                {
                    ActiveDirectionTarget = parallelObstacleDirection;
                }
            }
        }

        break;
    }

    case ObjectiveType::FLEEING:
        DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Fleeing", nullptr, FColor::Green, 0.0f, true);
        // If the agent does not detect the Obstacle in front of him.
        if (!AvoidObstacles(parallelObstacleDirection, forwardHit, target))
        {
            auto upVector = GetCharacter()->GetActorForwardVector();
            auto directionToTarget = GetCharacter()->GetActorLocation() - target;
            directionToTarget.Normalize();

            
            FHitResult hitData;
            // If the direction taken by the agent to flee doesn't hit a Obstacle.
            if (!DetectObstacles(hitData, directionToTarget, ForwardObstacleRayCastDist))
            {
                ActiveDirectionTarget = GetCharacter()->GetActorLocation() - target;
                ActiveDirectionTarget.Normalize();
            }
            // If the direction taken hit a Obstacle, we choose the parallel direction to the Obstacle that go away from the player.
            else
            {
                auto parallelHitDirection = hitData.ImpactNormal.Cross(upVector);
                if (parallelHitDirection.Dot(directionToTarget) <= 0)
                {
                    ActiveDirectionTarget = -parallelHitDirection;
                }
                else
                {
                    ActiveDirectionTarget = parallelHitDirection;
                }
            }
        }
        // If the agent detect a Obstacle, we follow the direction parallelObstacleDirection determined by AvoidObstacles.
        if (parallelObstacleDirection != FVector::ZeroVector)
        {
            hasForwardHit = true;
            ActiveDirectionTarget = parallelObstacleDirection.GetSafeNormal();
        }
        break;

    case ObjectiveType::WALKING:
        DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Walking", nullptr, FColor::Green, 0.0f, true);
        hasForwardHit = AvoidObstacles(parallelObstacleDirection, forwardHit, target);
        if (parallelObstacleDirection != FVector::ZeroVector)
        {
            ActiveDirectionTarget = parallelObstacleDirection;
        }
        break;
    }

    SpeedControl(deltaTime, hasForwardHit, forwardHit);
    Move(deltaTime, hasForwardHit, forwardHit);

    // debug print
    auto character = GetCharacter();
    DrawDebugPoint(GetWorld(), TargetMoveTo, 10.0f, FColor::Red);
    if (ActiveDirectionTarget != FVector::ZeroVector)
    {
        DrawDebugDirectionalArrow(GetWorld(), GetCharacter()->GetActorLocation(), GetCharacter()->GetActorLocation() + ActiveDirectionTarget * 100.0f, 2.0f, FColor::Magenta, false, -1.0f, 0U, 5.0f);
    }
}

bool ASDTAIController::AvoidObstacles(FVector &targetDirection, FHitResult &forwardHit, FVector playerPos)
{
    auto world = GetWorld();
    auto character = GetCharacter();
    auto collisionShape = character->GetCapsuleComponent()->GetCollisionShape();
    auto pos = character->GetActorLocation();
    auto forwardVector = character->GetActorForwardVector();

    auto isForwardHit = DetectObstacles(forwardHit, forwardVector, ForwardObstacleRayCastDist);

    if (isForwardHit)
    {
        // Calculate the cross product to get a horizontal vector on the plane of the obstacle pointing to the right.
        auto upVector = character->GetActorUpVector();
        auto parallelHitDirection = forwardHit.ImpactNormal.Cross(upVector);

        // For the first impact normal encountered, take one decision on the rotation side and stick to it.
        // For each subsequent different impact normal encountered during the rotation, determine another target direction,
        // but stick to the same rotation side to avoid to stay stuck in a corner.
        if (!LastImpactNormal.Equals(forwardHit.ImpactNormal))
        {
            auto isCornerDetected = LastImpactNormal != FVector::ZeroVector;
            LastImpactNormal = forwardHit.ImpactNormal;

            if (isCornerDetected)
            {
                // Stick on the same rotation side when detecting any subsequent obstacles. This is to avoid to stay stuck in a corner.
                targetDirection = IntRotationDirection() * parallelHitDirection;
            }
            else
            {
                // Calculate which direction (left or right) we want to go.
                // Add 1 cm to the capsule radius to do not be too close of the obstacle when doing the parallel SweepCast.
                auto impactPointWithCapsule = forwardHit.ImpactPoint + forwardHit.ImpactNormal * (collisionShape.GetCapsuleRadius() + 1.0f);

                // Use the horizontal vector parallel to the obstacle to detect another obstacle at the left or the right of the character.
                // It works here because the floor is flat. On an inclined floor, the sweep will hit the floor.
                FHitResult parallelHitSide1;
                auto isParallelHitSide1 = SDTUtils::SweepOverlapAgent(world, impactPointWithCapsule, impactPointWithCapsule + parallelHitDirection * SidesObstacleRayCastDist, collisionShape, parallelHitSide1);
                FHitResult parallelHitSide2;
                auto isParallelHitSide2 = SDTUtils::SweepOverlapAgent(world, impactPointWithCapsule, impactPointWithCapsule - parallelHitDirection * SidesObstacleRayCastDist, collisionShape, parallelHitSide2);

                auto productForwardAndNormal = parallelHitDirection.Dot(forwardVector);
                if (isParallelHitSide1 && isParallelHitSide2)
                {
                    // Turn to the direction where there is more space between the obstacles.
                    if (parallelHitSide1.Distance > parallelHitSide2.Distance)
                    {
                        RotationDirection = RotationSide::CLOCKWISE;
                    }
                    else
                    {
                        RotationDirection = RotationSide::COUNTER_CLOCKWISE;
                    }
                }
                else if (isParallelHitSide1)
                {
                    RotationDirection = RotationSide::COUNTER_CLOCKWISE;
                }
                else if (isParallelHitSide2)
                {
                    RotationDirection = RotationSide::CLOCKWISE;
                }
                // If the actor approximately faces the obstacle, choose a random direction.
                else if (-0.05f < productForwardAndNormal && productForwardAndNormal < 0.05f)
                {
                    //If the actor flee a player, we choose de rotation which will give the direction to go far away from the player.
                    if (playerPos != FVector::ZeroVector)
                    {
                        auto directionPlayerToAgent = pos - playerPos;
                        if (parallelHitDirection.Dot(directionPlayerToAgent) <= 0)
                        {
                            RotationDirection = RotationSide::COUNTER_CLOCKWISE;
                        }
                        else
                        {
                            RotationDirection = RotationSide::CLOCKWISE;
                        }
                    }
                    else
                    {
                        RotationDirection = FMath::RandBool() ? RotationSide::CLOCKWISE : RotationSide::COUNTER_CLOCKWISE;
                    }
                }
                else
                {
                    //If the actor flee a player, we choose de rotation which will give the direction to go far away from the player.
                    if (playerPos != FVector::ZeroVector)
                    {
                        auto directionPlayerToAgent = pos - playerPos;
                        if (parallelHitDirection.Dot(directionPlayerToAgent) <= 0)
                        {
                            RotationDirection = RotationSide::COUNTER_CLOCKWISE;
                        }
                        else
                        {
                            RotationDirection = RotationSide::CLOCKWISE;
                        }
                    }
                    else
                    {
                        //The agent will go to the right/left of the obstacle if it comes from the right/left
                        RotationDirection = productForwardAndNormal >= 0 ? RotationSide::CLOCKWISE : RotationSide::COUNTER_CLOCKWISE;
                    }
                }
                targetDirection = IntRotationDirection() * parallelHitDirection;

                DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule + forwardVector * ForwardObstacleRayCastDist, ColorIfHit(isForwardHit), false, collisionDurationDebug, 0, ThicknessIfHit(isForwardHit));
                DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule + parallelHitDirection * SidesObstacleRayCastDist, ColorIfHit(isParallelHitSide1), false, collisionDurationDebug, 0, ThicknessIfHit(isParallelHitSide1));
                DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule - parallelHitDirection * SidesObstacleRayCastDist, ColorIfHit(isParallelHitSide2), false, collisionDurationDebug, 0, ThicknessIfHit(isParallelHitSide2));
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + forwardHit.ImpactNormal * 100.0), 5.0f, FColor::Green, false, collisionDurationDebug, 0, 2.0f);
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + upVector * 100.0), 5.0f, FColor::Blue, false, collisionDurationDebug, 0, 2.0f);
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + parallelHitDirection * 100.0), 5.0f, FColor::Magenta, false, collisionDurationDebug, 0, 2.0f);
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint - forwardVector * 100.0, forwardHit.ImpactPoint, 5.0f, FColor::Magenta, false, collisionDurationDebug, 0, 2.0f);
                auto hitComponent = forwardHit.GetComponent();
                DrawDebugBox(world, hitComponent->Bounds.Origin, hitComponent->Bounds.BoxExtent, FColor::Green, false, collisionDurationDebug);
            }
        }
        
        //If the agent hit the same obstacle
        else
        {
            //If the agent flees a player, we make sure the targetDirection allow to the agent to go far from the player.
            if (playerPos != FVector::ZeroVector)
            {
                auto directionPlayerToAgent = pos - playerPos;
                if (parallelHitDirection.Dot(directionPlayerToAgent) <= 0)
                {
                    RotationDirection = RotationSide::COUNTER_CLOCKWISE;
                }
            }
            //We keep the same direction as before
            targetDirection = IntRotationDirection() * parallelHitDirection;
        }

        return true;
    }
    else
    {
        LastImpactNormal = FVector::ZeroVector;
    }

    return false;
}
bool ASDTAIController::DetectObstacles(FHitResult &hitData, FVector hitDirection, float hitDist)
{
    auto world = GetWorld();
    auto character = GetCharacter();
    auto collisionShape = character->GetCapsuleComponent()->GetCollisionShape();

    auto forwardVector = hitDirection;
    auto loc = character->GetActorLocation();

    return SDTUtils::SweepOverlapAgent(world, loc, loc + hitDist * forwardVector, collisionShape, hitData);
}

void ASDTAIController::ResetObstaclesDetection()
{
    LastImpactNormal = FVector::ZeroVector;
}

void ASDTAIController::UpdateTargetPositionOnSpline(float deltaTime)
{
    auto totalSplineLength = SplineChassing->GetSplineLength();
    SplineDistance = SplineDistance + GetCharacter()->GetCharacterMovement()->MaxWalkSpeed * deltaTime;
    if (SplineDistance >= totalSplineLength)
    {
        SplineDistance = -1.0f;
        ActiveDirectionTarget = GetCharacter()->GetActorForwardVector();
    }
}

void ASDTAIController::Move(float deltaTime, bool hasForwardHit, const FHitResult &forwardHit)
{
    auto character = GetCharacter();
    if (SplineDistance == -1.0f)
    {
        auto forward = character->GetActorForwardVector();
        if (ActiveDirectionTarget != FVector::ZeroVector)
        {
            if (FVector::Parallel(forward, ActiveDirectionTarget.ProjectOnToNormal(forward), 0.990f) && forward.Dot(ActiveDirectionTarget) > 0)
            {
                CalculateFarForwardTarget(ActiveDirectionTarget);
                ActiveDirectionTarget = FVector::ZeroVector;
            }
            else
            {
                auto doRotation = true;

                auto rotationAxis = forward.Cross(ActiveDirectionTarget);
                if (rotationAxis.IsNearlyZero())
                {
                    rotationAxis = character->GetActorUpVector();
                }

                // Locate a trap beside the agent (at the left or at the right) at a distance relative to the distance between the agent capsule and the obstacle in front of the agent.
                // If a trap is located, stop the rotation to go closer to the obstacle before rotating again.
                if (hasForwardHit && GetCharacter()->GetCharacterMovement()->GetMaxSpeed() > 0.0f)
                {
                    auto capsule = character->GetCapsuleComponent();
                    auto capsuleRadius = capsule->GetScaledCapsuleRadius();
                    // Calculate the closest point from the capsule on the obstacle. We take a line passing at the character location in the normal direction.
                    auto closestPointOnObstacle = FMath::LinePlaneIntersection(character->GetActorLocation(), character->GetActorLocation() - forwardHit.ImpactNormal, forwardHit.ImpactPoint, forwardHit.ImpactNormal);
                    // Calculate the distance at which we must look beside the agent for the trap. The distance depends of the distance from the obstacle.
                    auto sideDistance = FMath::Clamp(((closestPointOnObstacle - character->GetActorLocation()).Size() - capsuleRadius), 0, 4 * capsuleRadius);
                    // We need to know if the orientation is clockwise or counter-clockwise to determine if we must overlap on the left or the right side.
                    auto rotationDirection = rotationAxis.Dot(character->GetActorUpVector()) >= 0 ? 1 : -1;
                    auto capsuleCenter = character->GetActorLocation() + rotationDirection * sideDistance * character->GetActorRightVector();
                    // We will rotate the agent if there is no overlap of a trap beside.
                    doRotation = !GetWorld()->OverlapAnyTestByObjectType(capsuleCenter, FQuat::Identity, ECC_TO_BITFIELD(COLLISION_DEATH_OBJECT), character->GetCapsuleComponent()->GetCollisionShape());
                    DrawDebugCapsule(GetWorld(), capsuleCenter, capsule->GetScaledCapsuleHalfHeight(), capsule->GetScaledCapsuleRadius(), FQuat::Identity, FColor::Blue, false, -1.0f, 0U, 3.0f);
                }

                if (doRotation)
                {
                    DrawDebugDirectionalArrow(GetWorld(), character->GetActorLocation(), character->GetActorLocation() + rotationAxis * 100.0f, 3.0f, FColor::Red, false, -1.0f, 0U, 6.0f);
                    auto nextDirection = forward.RotateAngleAxis(RotationAngleBySecond * deltaTime, rotationAxis);
                    CalculateFarForwardTarget(nextDirection);
                }
            }
        }
    }
    else
    {
        auto splineLocation = SplineChassing->GetWorldLocationAtDistanceAlongSpline(SplineDistance);
        SetTarget(splineLocation);
        DrawDebugPoint(GetWorld(), splineLocation, 10.0f, FColor::Magenta);
    }
}

void ASDTAIController::SpeedControl(float deltaTime, bool hasForwardHit, const FHitResult &forwardHit)
{
    auto character = GetCharacter();
    auto currentSpeed = character->GetVelocity().Size();
    // While the character is turning, reduce the speed below a maximum speed value if it is running too fast, or just keep its speed constant if below that maximum speed.
    float accelerationToApply;
    if (hasForwardHit)
    {
        if (forwardHit.Distance <= 100.0f)
        {
            accelerationToApply = -2 * Acceleration;
        }
        else if (currentSpeed > 200)
        {
            accelerationToApply = -Acceleration;
        }
        else
        {
            accelerationToApply = 0;
        }
    }
    else
    {
        accelerationToApply = Acceleration;
    }

    // Restrict the max walk speed to never exceed configuration variable MaxSpeed and to never go below 0.
    character->GetCharacterMovement()->MaxWalkSpeed = FMath::Clamp(currentSpeed + accelerationToApply * deltaTime, 0.0f, MaxSpeed);
}

void ASDTAIController::DetectObjective(ObjectiveType &objective, FVector &target)
{
    auto location = GetCharacter()->GetActorLocation();
    TArray<FOverlapResult> overlapData;
    if (SDTUtils::DetectTargetsFromAgent(GetWorld(), location, VisionDistance * GetCharacter()->GetActorForwardVector(), VisionAngle, overlapData))
    {
        auto overlap = overlapData.FindByPredicate([](auto overlap) -> auto
                                                   { return overlap.GetComponent()->GetCollisionObjectType() == COLLISION_PLAYER; });
        if (overlap != nullptr)
        {
            // Chase or flee from the player.
            objective = SDTUtils::IsPlayerPoweredUp(GetWorld()) ? ObjectiveType::FLEEING : ObjectiveType::CHASSING;

            target = overlap->GetActor()->GetActorLocation();
        }
        else
        {
            // Chase the closest collectible, or just walk if all collectibles are hidden.
            objective = ObjectiveType::CHASSING;
            auto smallestDistance = INFINITY;
            for (int i = 0; i < overlapData.Num(); ++i)
            {
                auto collectible = dynamic_cast<ASDTCollectible *>(overlapData[i].GetActor());
                if (collectible != nullptr && !collectible->IsOnCooldown())
                {
                    auto distance = (overlapData[i].GetActor()->GetActorLocation() - location).Size();
                    if (distance < smallestDistance)
                    {
                        smallestDistance = distance;
                        target = overlapData[i].GetActor()->GetActorLocation();
                    }
                }
            }
            objective = smallestDistance != INFINITY ? ObjectiveType::CHASSING : ObjectiveType::WALKING;
        }
    }
    else
    {
        objective = ObjectiveType::WALKING;
    }

    DrawDebugCone(GetWorld(), location, GetCharacter()->GetActorForwardVector(), VisionDistance, FMath::DegreesToRadians(VisionAngle), FMath::DegreesToRadians(VisionAngle), 24, FColor::Red);
    DrawDebugSphere(GetWorld(), location, VisionDistance, 24, FColor::Green);
}
