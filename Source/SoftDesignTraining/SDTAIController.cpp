// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTAIController.h"
#include "SoftDesignTraining.h"
#include "SDTUtils.h"
#include "SDTCollectible.h"

const float collisionDurationDebug = 5.0f;


void ASDTAIController::CalculateFarForwardTarget(FVector headingTarget)
{
    auto character = GetCharacter();
    float distance = 10000.0f;
    FHitResult hitResult;

    targetMoveTo = GetCharacter()->GetActorLocation() + distance * headingTarget;
    if (SDTUtils::BlockingRayAgent(GetWorld(), character->GetActorLocation(), targetMoveTo, hitResult))
    {
        // This is to ensure to set a target point and to avoid an infinite recursion between MoveToLocation and OnMoveCompleted.
        distance = FMath::Max(hitResult.Distance, 200);
        targetMoveTo = GetCharacter()->GetActorLocation() + distance * headingTarget;
    }
    MoveToLocation(targetMoveTo, -1, true, false, false);
}

void ASDTAIController::CalculateFarForwardTarget()
{
    CalculateFarForwardTarget(GetCharacter()->GetActorForwardVector());
}

void ASDTAIController::BeginPlay()
{
    Super::BeginPlay();
    chassingSpline = Cast<USplineComponent>(AddComponentByClass(USplineComponent::StaticClass(), false, FTransform::Identity, false));
    chassingSpline->ClearSplinePoints();
    chassingSpline->SetDrawDebug(true);
    auto character = GetCharacter();
    auto moveComp = character->GetCharacterMovement();
    moveComp->MaxAcceleration = Acceleration;
    CalculateFarForwardTarget();
}


void ASDTAIController::OnMoveCompleted(FAIRequestID RequestID, EPathFollowingResult::Type Result)
{
    if (Result != EPathFollowingResult::Aborted)
    {
        CalculateFarForwardTarget();
    }
}

void ASDTAIController::Tick(float deltaTime)
{
    Super::Tick(deltaTime);
    FVector targetDirection = FVector::ZeroVector;
    bool followSpline = false;
    float wallCollisionDistance = -1.0f;
    FVector target = FVector::ZeroVector;
    ObjectiveType objective;

    if (splineDistance != -1.0f && splineDistance <= chassingSpline->GetSplineLength() / 2)
    {
        followSpline = true;
    }
    else
    {
        DetectObjective(objective, target);
        switch (objective)
        {
        case ObjectiveType::CHASSING:
        {
            ResetWallsDetection();
            TArray<FSplinePoint> pathPoints;
            if (SDTUtils::FindPathToLocation(GetWorld(), GetCharacter()->GetActorLocation(), target, GetCharacter()->GetActorUpVector(), GetCharacter()->GetCapsuleComponent(), pathPoints))
            {
                if (pathPoints.Num() <= 2)
                {
                    DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Chassing direct", nullptr, FColor::Green, 0.0f, true);
                    targetDirection = target - GetCharacter()->GetActorLocation();
                    targetDirection.Normalize();
                }
                else
                {
                    DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Chassing spline", nullptr, FColor::Green, 0.0f, true);
                    for (auto path : pathPoints)
                    {
                        DrawDebugPoint(GetWorld(), path.Position, 5.0f, FColor::Green, false, 10.0f);
                    }
                    followSpline = true;
                    chassingSpline->ClearSplinePoints(false);
                    chassingSpline->AddPoints(pathPoints);
                    splineDistance = 0.0f;
                }
            }
            else
            {
                // TODO: Move to the closest point before the collision in straight line?
            }
            break;
        }

        case ObjectiveType::FLEEING:
        {
            DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Fleeing", nullptr, FColor::Green, 0.0f, true);

            chassingSpline->ClearSplinePoints(true);
            splineDistance = -1.0f;

            FVector parallelWallDirection;
            FVector fleeDirection = target - GetCharacter()->GetActorLocation();
            fleeDirection.Normalize();
            if (DetectWalls(parallelWallDirection, wallCollisionDistance))
            {
                if (fleeDirection.Dot(parallelWallDirection) < 0)
                {
                    fleeDirection = -parallelWallDirection;
                }
            }
        }
        break;

        case ObjectiveType::WALKING:
            DrawDebugString(GetWorld(), GetCharacter()->GetActorLocation(), "Walking", nullptr, FColor::Green, 0.0f, true);

            chassingSpline->ClearSplinePoints(true);
            splineDistance = -1.0f;

            DetectWalls(targetDirection, wallCollisionDistance);
            break;
        }
    }

    SpeedControl(deltaTime, wallCollisionDistance);
    if (followSpline)
    {
        Move(deltaTime);
    }
    else
    {
        Move(deltaTime, targetDirection);
    }
}

bool ASDTAIController::DetectWalls(FVector& targetDirection, float& collisionDistance)
{
    auto world = GetWorld();
    auto character = GetCharacter();
    auto collisionShape = character->GetCapsuleComponent()->GetCollisionShape();

    auto forwardVector = character->GetActorForwardVector();
    auto loc = character->GetActorLocation();

    TArray<FHitResult> hitData;
    SDTUtils::SweepOverlapAgent(world, loc, loc + ForwardWallRayCastDist * forwardVector, collisionShape, hitData);
    bool isForwardHit = !hitData.IsEmpty();

    if (isForwardHit)
    {
        FHitResult& forwardHit = hitData[0];
        collisionDistance = forwardHit.Distance;

        // For the first impact normal encountered, take one decision on the rotation side and stick to it.
        // For each subsequent different impact normal encountered during the rotation, determine another target direction,
        // but stick to the same rotation side to avoid to stay stuck in a corner.
        if (!lastImpactNormal.Equals(forwardHit.ImpactNormal))
        {
            GEngine->AddOnScreenDebugMessage(INDEX_NONE, 1.0f, FColor::Blue, FString::Printf(TEXT("LastNormal: %s, NewNormal: %s, Angle is: %f"), *lastImpactNormal.ToString(), *forwardHit.ImpactNormal.ToString(), SDTUtils::CosineVectors(lastImpactNormal, forwardHit.ImpactNormal)));
            bool isNewWallDetection = lastImpactNormal == FVector::ZeroVector;
            lastImpactNormal = forwardHit.ImpactNormal;

            // Calculate the cross product to get a horizontal vector on the plane of the wall pointing to the right.
            auto upVector = character->GetActorUpVector();
            auto parallelHitDirection = forwardHit.ImpactNormal.Cross(upVector);

            if (!isNewWallDetection)
            {
                // Stick on the same rotation side when detecting any subsequent walls. This is to avoid to stay stuck in a corner.
                lastTargetDirectionForWalls = rotationDirection * parallelHitDirection;
            }
            else
            {
                // Calculate which direction (left or right) we want to go.
                // Add 1 cm to the capsule radius to do not be too close of the wall when doing the parallel SweepCast.
                FVector impactPointWithCapsule = forwardHit.ImpactPoint + forwardHit.ImpactNormal * (collisionShape.GetCapsuleRadius() + 1.0f);

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
                        rotationDirection = 1;
                        //GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Walls on both sides. Direction is ") + FString::FromInt(rotationDirection));
                    }
                    else
                    {
                        rotationDirection = -1;
                        //GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Walls on both sides. Direction is ") + FString::FromInt(rotationDirection));
                    }
                }
                else if (isParallelHitSide1)
                {
                    rotationDirection = -1;
                    //GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Wall on right side only. Direction is ") + FString::FromInt(rotationDirection));
                }
                else if (isParallelHitSide2)
                {
                    rotationDirection = 1;
                    //GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Wall on left side only. Direction is ") + FString::FromInt(rotationDirection));
                }
                // If the actor approximately faces the wall, choose a random direction.
                else if (-0.05f < productForwardAndNormal && productForwardAndNormal < 0.05f)
                {
                    rotationDirection = FMath::RandBool() ? 1 : -1;
                    //GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Random Direction Chosen is ") + FString::FromInt(rotationDirection));
                }
                else
                {
                    lastTargetDirectionForWalls = FMath::Sign(productForwardAndNormal) * parallelHitDirection;
                    rotationDirection = FMath::Sign(productForwardAndNormal);
                    //GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Direction is ") + FString::FromInt(rotationDirection));
                }
                lastTargetDirectionForWalls = rotationDirection * parallelHitDirection;


                DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule + forwardVector * ForwardWallRayCastDist, FColor::Red, false, collisionDurationDebug, 0, isForwardHit ? 5.0f : 0.0f);
                DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule + parallelHitDirection * SidesWallRayCastDist, FColor::Red, false, collisionDurationDebug, 0, isParallelHitSide1 ? 5.0f : 0.0f);
                DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule - parallelHitDirection * SidesWallRayCastDist, FColor::Red, false, collisionDurationDebug, 0, isParallelHitSide2 ? 5.0f : 0.0f);
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + forwardHit.ImpactNormal * 100.0), 5.0f, FColor::Green, false, collisionDurationDebug, 0, 2.0f);
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + upVector * 100.0), 5.0f, FColor::Blue, false, collisionDurationDebug, 0, 2.0f);
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + parallelHitDirection * 100.0), 5.0f, FColor::Magenta, false, collisionDurationDebug, 0, 2.0f);
                DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint - forwardVector * 100.0, forwardHit.ImpactPoint, 5.0f, FColor::Magenta, false, collisionDurationDebug, 0, 2.0f);
                UPrimitiveComponent* hitComponent = forwardHit.GetComponent();
                DrawDebugBox(world, hitComponent->Bounds.Origin, hitComponent->Bounds.BoxExtent, FColor::Green, false, collisionDurationDebug);
            }
        }
        targetDirection = lastTargetDirectionForWalls;

        return true;
    }
    else if (!forwardVector.Equals(lastTargetDirectionForWalls))
    {
        targetDirection = lastTargetDirectionForWalls;
        return true;
    }
    else
    {
        lastTargetDirectionForWalls = FVector::ZeroVector;
        lastImpactNormal = FVector::ZeroVector;
    }

    return false;
}

void ASDTAIController::ResetWallsDetection()
{
    lastImpactNormal = FVector::ZeroVector;
    lastTargetDirectionForWalls = FVector::ZeroVector;
}

void ASDTAIController::Move(float deltaTime, FVector targetDirection)
{
    auto character = GetCharacter();
    FVector forward = character->GetActorForwardVector();
    if (targetDirection != FVector::ZeroVector)
    {
        //float angle = SDTUtils::CosineVectors(forward, targetDirection);
        if (FVector::Parallel(forward, targetDirection) && forward.Dot(targetDirection) > 0)
        {
            CalculateFarForwardTarget(targetDirection);
            previousRotationAxis = FVector::ZeroVector;
        }
        else
        {
            FVector rotationAxis = forward.Cross(targetDirection);
            if (rotationAxis.IsNearlyZero())
            {
                rotationAxis = character->GetActorUpVector();
            }
            else
            {
                rotationAxis.Normalize();
            }
            DrawDebugDirectionalArrow(GetWorld(), character->GetActorLocation(), character->GetActorLocation() + rotationAxis * 100.0f, 3.0f, FColor::Red, false, -1.0f, 0U, 6.0f);
            FVector nextDirection = forward.RotateAngleAxis(RotationAngleBySecond * deltaTime, rotationAxis);
            CalculateFarForwardTarget(nextDirection);
            previousRotationAxis = rotationAxis;
        }
    }

    // debug print
    DrawDebugPoint(GetWorld(), targetMoveTo, 5.0f, FColor::Red);
    if (targetDirection != FVector::ZeroVector)
    {
        DrawDebugDirectionalArrow(GetWorld(), character->GetActorLocation(), character->GetActorLocation() + targetDirection * 100.0f, 1.0f, FColor::Blue, false, -1.0f, 0U, 6.0f);
    }
    GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Yellow, FString::Printf(TEXT("[%s] Velocity: %f cm/s"), *character->GetName(), character->GetVelocity().Size()));
}

void ASDTAIController::Move(float deltaTime)
{
    splineDistance = FMath::Min(chassingSpline->GetSplineLength(), splineDistance + GetCharacter()->GetCharacterMovement()->MaxWalkSpeed * deltaTime);
    auto splineLocation = chassingSpline->GetWorldLocationAtDistanceAlongSpline(splineDistance);
    MoveToLocation(splineLocation, -1, true, false, false);

    GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Black, FString::Printf(TEXT("splineLocation: %s"), *splineLocation.ToString()));
    GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Black, FString::Printf(TEXT("splineDistance: %f"), splineDistance));
    GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Black, FString::Printf(TEXT("splineLength: %f"), chassingSpline->GetSplineLength()));
    DrawDebugPoint(GetWorld(), splineLocation, 10.0f, FColor::Blue);
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
            accelerationToApply = - 2 * Acceleration;
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


void ASDTAIController::DetectObjective(ObjectiveType& objective, FVector& target)
{
    FVector location = GetCharacter()->GetActorLocation();
    TArray<FOverlapResult> overlapData;
    if (SDTUtils::DetectTargetsFromAgent(GetWorld(), location, VisionDistance * GetCharacter()->GetActorForwardVector(), VisionAngle, overlapData))
    {
        auto overlap = overlapData.FindByPredicate([](auto overlap) -> auto {
            return overlap.GetComponent()->GetCollisionObjectType() == COLLISION_PLAYER;
        });
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
                ASDTCollectible *collectible = dynamic_cast<ASDTCollectible*>(overlapData[i].GetActor());
                if (collectible != nullptr && !collectible->IsOnCooldown()) {
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
