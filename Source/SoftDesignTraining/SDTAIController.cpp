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

    auto character = GetCharacter();
    auto moveComp = character->GetCharacterMovement();
    moveComp->MaxWalkSpeed = MaxSpeed;
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
    auto wallCollisionDistance = -1.0f;
    auto target = FVector::ZeroVector;
    auto parallelWallDirection = FVector::ZeroVector;
    ObjectiveType objective;

    DetectObjective(objective, target);
    if (objective != CurrentObjective)
    {
        switch (CurrentObjective)
        {
        case ObjectiveType::CHASSING:
            SplineChassing->ClearSplinePoints(true);
            SplineDistance = -1.0f;
            break;

        case ObjectiveType::WALKING:
            ResetWallsDetection();
            break;

        case ObjectiveType::FLEEING:
            ResetWallsDetection();
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
            TArray<FHitResult> obstacles;
            // Check if we have a direct path to the target. If yes, stop following the spline.
            if (!SDTUtils::SweepOverlapAgent(GetWorld(), GetCharacter()->GetActorLocation(), target, GetCharacter()->GetCapsuleComponent()->GetCollisionShape(), obstacles))
            {
                DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Chassing direct", nullptr, FColor::Green, 0.0f, true);
                SplineDistance = -1.0f;
                ActiveDirectionTarget = target - GetCharacter()->GetActorLocation();
                ActiveDirectionTarget.Normalize();
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
                    ActiveDirectionTarget.Normalize();
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
                DetectWalls(parallelWallDirection, wallCollisionDistance);
                if (parallelWallDirection != FVector::ZeroVector)
                {
                    ActiveDirectionTarget = parallelWallDirection;
                }
            }
        }

        break;
    }

    case ObjectiveType::FLEEING:
        DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Fleeing", nullptr, FColor::Green, 0.0f, true);
        DetectWalls(parallelWallDirection, wallCollisionDistance);
        ActiveDirectionTarget = GetCharacter()->GetActorLocation() - target;
        ActiveDirectionTarget.Normalize();
        if (DetectWalls(parallelWallDirection, wallCollisionDistance))
        {
            if (ActiveDirectionTarget.Dot(parallelWallDirection) < 0)
            {
                ActiveDirectionTarget = -parallelWallDirection;
            }
        }
        break;

    case ObjectiveType::WALKING:
        DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Walking", nullptr, FColor::Green, 0.0f, true);
        DetectWalls(parallelWallDirection, wallCollisionDistance);
        if (parallelWallDirection != FVector::ZeroVector)
        {
            ActiveDirectionTarget = parallelWallDirection;
        }
        break;
    }

    SpeedControl(deltaTime, wallCollisionDistance);
    Move(deltaTime);

    // debug print
    auto character = GetCharacter();
    DrawDebugPoint(GetWorld(), TargetMoveTo, 10.0f, FColor::Red);
    if (ActiveDirectionTarget != FVector::ZeroVector)
    {
        DrawDebugDirectionalArrow(GetWorld(), character->GetActorLocation(), character->GetActorLocation() + ActiveDirectionTarget * 100.0f, 1.0f, FColor::Blue, false, -1.0f, 0U, 6.0f);
    }
    GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Yellow, FString::Printf(TEXT("[%s] Velocity: %f cm/s"), *character->GetName(), character->GetVelocity().Size()));
}

bool ASDTAIController::DetectWalls(FVector &targetDirection, float &collisionDistance)
{
    auto world = GetWorld();
    auto character = GetCharacter();
    auto collisionShape = character->GetCapsuleComponent()->GetCollisionShape();

    auto forwardVector = character->GetActorForwardVector();
    auto loc = character->GetActorLocation();

    TArray<FHitResult> hitData;
    SDTUtils::SweepOverlapAgent(world, loc, loc + ForwardWallRayCastDist * forwardVector, collisionShape, hitData);
    auto isForwardHit = !hitData.IsEmpty();

    if (isForwardHit)
    {
        auto& forwardHit = hitData[0];
        collisionDistance = forwardHit.Distance;

        // For the first impact normal encountered, take one decision on the rotation side and stick to it.
        // For each subsequent different impact normal encountered during the rotation, determine another target direction,
        // but stick to the same rotation side to avoid to stay stuck in a corner.
        if (!LastImpactNormal.Equals(forwardHit.ImpactNormal))
        {
            // GEngine->AddOnScreenDebugMessage(INDEX_NONE, 1.0f, FColor::Blue, FString::Printf(TEXT("LastNormal: %s, NewNormal: %s, Angle is: %f"), *lastImpactNormal.ToString(), *forwardHit.ImpactNormal.ToString(), SDTUtils::CosineVectors(lastImpactNormal, forwardHit.ImpactNormal)));
            bool isNewWallDetection = LastImpactNormal == FVector::ZeroVector;
            LastImpactNormal = forwardHit.ImpactNormal;

            // Calculate the cross product to get a horizontal vector on the plane of the wall pointing to the right.
            auto upVector = character->GetActorUpVector();
            auto parallelHitDirection = forwardHit.ImpactNormal.Cross(upVector);

            if (!isNewWallDetection)
            {
                // Stick on the same rotation side when detecting any subsequent walls. This is to avoid to stay stuck in a corner.
                targetDirection = RotationDirection * parallelHitDirection;
            }
            else
            {
                // Calculate which direction (left or right) we want to go.
                // Add 1 cm to the capsule radius to do not be too close of the wall when doing the parallel SweepCast.
                auto impactPointWithCapsule = forwardHit.ImpactPoint + forwardHit.ImpactNormal * (collisionShape.GetCapsuleRadius() + 1.0f);

                // Use the horizontal vector parallel to the wall to detect another wall at the left or the right of the character.
                // It works here because the floor is flat. On an inclined floor, the sweep will hit the floor.
                TArray<FHitResult> parallelHitSide1;
                auto isParallelHitSide1 = SDTUtils::SweepOverlapAgent(world, impactPointWithCapsule, impactPointWithCapsule + parallelHitDirection * SidesWallRayCastDist, collisionShape, parallelHitSide1);
                TArray<FHitResult> parallelHitSide2;
                auto isParallelHitSide2 = SDTUtils::SweepOverlapAgent(world, impactPointWithCapsule, impactPointWithCapsule - parallelHitDirection * SidesWallRayCastDist, collisionShape, parallelHitSide2);

                auto productForwardAndNormal = parallelHitDirection.Dot(forwardVector);
                if (!parallelHitSide1.IsEmpty() && !parallelHitSide2.IsEmpty())
                {
                    // Turn to the direction where there is more space between the walls.
                    if (parallelHitSide1[0].Distance > parallelHitSide2[0].Distance)
                    {
                        RotationDirection = 1;
                    }
                    else
                    {
                        RotationDirection = -1;
                    }
                }
                else if (isParallelHitSide1)
                {
                    RotationDirection = -1;
                }
                else if (isParallelHitSide2)
                {
                    RotationDirection = 1;
                }
                // If the actor approximately faces the wall, choose a random direction.
                else if (-0.05f < productForwardAndNormal && productForwardAndNormal < 0.05f)
                {
                    RotationDirection = FMath::RandBool() ? 1 : -1;
                }
                else
                {
                    RotationDirection = FMath::Sign(productForwardAndNormal);
                }
                targetDirection = RotationDirection * parallelHitDirection;

                DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule + forwardVector * ForwardWallRayCastDist, ColorIfHit(isForwardHit), false, collisionDurationDebug, 0, ThicknessIfHit(isForwardHit));
                DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule + parallelHitDirection * SidesWallRayCastDist, ColorIfHit(isParallelHitSide1), false, collisionDurationDebug, 0, ThicknessIfHit(isParallelHitSide1));
                DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule - parallelHitDirection * SidesWallRayCastDist, ColorIfHit(isParallelHitSide2), false, collisionDurationDebug, 0, ThicknessIfHit(isParallelHitSide2));
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + forwardHit.ImpactNormal * 100.0), 5.0f, FColor::Green, false, collisionDurationDebug, 0, 2.0f);
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + upVector * 100.0), 5.0f, FColor::Blue, false, collisionDurationDebug, 0, 2.0f);
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + parallelHitDirection * 100.0), 5.0f, FColor::Magenta, false, collisionDurationDebug, 0, 2.0f);
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint - forwardVector * 100.0, forwardHit.ImpactPoint, 5.0f, FColor::Magenta, false, collisionDurationDebug, 0, 2.0f);
                auto hitComponent = forwardHit.GetComponent();
                DrawDebugBox(world, hitComponent->Bounds.Origin, hitComponent->Bounds.BoxExtent, FColor::Green, false, collisionDurationDebug);
            }
        }

        return true;
    }
    else
    {
        LastImpactNormal = FVector::ZeroVector;
    }

    return false;
}

void ASDTAIController::ResetWallsDetection()
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

void ASDTAIController::Move(float deltaTime)
{
    auto character = GetCharacter();
    if (SplineDistance == -1.0f)
    {
        auto forward = character->GetActorForwardVector();
        if (ActiveDirectionTarget != FVector::ZeroVector)
        {
            if (FVector::Parallel(forward, ActiveDirectionTarget) && forward.Dot(ActiveDirectionTarget) > 0)
            {
                CalculateFarForwardTarget(ActiveDirectionTarget);
                ActiveDirectionTarget = FVector::ZeroVector;
            }
            else
            {
                auto rotationAxis = forward.Cross(ActiveDirectionTarget);
                if (rotationAxis.IsNearlyZero())
                {
                    rotationAxis = character->GetActorUpVector();
                }
                else
                {
                    rotationAxis.Normalize();
                }
                DrawDebugDirectionalArrow(GetWorld(), character->GetActorLocation(), character->GetActorLocation() + rotationAxis * 100.0f, 3.0f, FColor::Red, false, -1.0f, 0U, 6.0f);
                auto nextDirection = forward.RotateAngleAxis(RotationAngleBySecond * deltaTime, rotationAxis);
                CalculateFarForwardTarget(nextDirection);
            }
        }
    }
    else
    {
        auto splineLocation = SplineChassing->GetWorldLocationAtDistanceAlongSpline(SplineDistance);
        SetTarget(splineLocation);

        // GEngine->AddOnScreenDebugMessage(6, 0.0f, FColor::Black, FString::Printf(TEXT("splineHeight: %f, character height: %f"), splineLocation.Z, GetCharacter()->GetActorLocation().Z));
        // GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Black, FString::Printf(TEXT("splineLocation: %s"), *splineLocation.ToString()));
        // GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Black, FString::Printf(TEXT("splineDistance: %f"), SplineDistance));
        // GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Black, FString::Printf(TEXT("splineLength: %f"), SplineChassing->GetSplineLength()));
        DrawDebugPoint(GetWorld(), splineLocation, 10.0f, FColor::Magenta);
    }
}

void ASDTAIController::SpeedControl(float deltaTime, float wallCollisionDistance)
{
    auto character = GetCharacter();
    auto currentSpeed = character->GetVelocity().Size();
    // While the character is turning, reduce the speed below a maximum speed value if it is running too fast, or just keep its speed constant if below that maximum speed.
    float accelerationToApply;
    if (wallCollisionDistance != -1)
    {
        if (wallCollisionDistance <= 100.0f)
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

    // TODO: Is the minimum max speed (0.01f) is necessary?
    // Always allow the character a really small max speed to allow it to turn around itself with the MoveTo calls.
    // Restrict the max walk speed to never exceed configuration variable MaxSpeed.
    character->GetCharacterMovement()->MaxWalkSpeed = FMath::Clamp(currentSpeed + accelerationToApply * deltaTime, 0.01f, MaxSpeed);
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
}
