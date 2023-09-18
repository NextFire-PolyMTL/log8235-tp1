// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTAIController.h"
#include "SoftDesignTraining.h"
#include "SDTUtils.h"

const float collisionDurationDebug = 5.0f;

void calculateSplineCurve(FVector startingPoint, float forwardDistance)
{

}

void ASDTAIController::CalculateFarForwardTarget(FVector headingTarget)
{
    auto character = GetCharacter();
    float distance = 10000.0f;
    FHitResult hitResult;

    targetMoveTo = GetCharacter()->GetActorLocation() + distance * headingTarget;
    if (SDTUtils::Raycast(GetWorld(), character->GetActorLocation(), targetMoveTo, hitResult))
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
    auto character = GetCharacter();
    auto moveComp = character->GetCharacterMovement();
    moveComp->MaxWalkSpeed = MaxSpeed;
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
    DetectWalls();
    //DetectCollectible();
    SpeedControl(deltaTime);
    Move(deltaTime);
}

void ASDTAIController::DetectWalls()
{
    auto world = GetWorld();
    auto character = GetCharacter();
    auto collisionShape = character->GetCapsuleComponent()->GetCollisionShape();

    auto forwardVector = character->GetActorForwardVector();
    auto loc = character->GetActorLocation();

    TArray<FHitResult> hitData;
    SDTUtils::SweepCast(world, loc, forwardVector, ForwardWallRayCastDist, collisionShape, hitData);
    isForwardHit = !hitData.IsEmpty();

    if (isForwardHit)
    {
        FHitResult& forwardHit = hitData[0];
        forwardImpactDistance = forwardHit.Distance;
        lastImpactNormal = forwardHit.ImpactNormal;

        if (!isTurningAround)
        {
            // Take only one decision on the side to turn around and stick on this rotation side until no
            // forward collision is detected. This is to avoid to stay stuck in a corner.
            isTurningAround = true;

            // Calculate the cross product to get a horizontal vector on the plane of the wall pointing to the right.
            auto upVector = character->GetActorUpVector();
            auto parallelHitDirection = forwardHit.ImpactNormal.Cross(upVector);

            // Add 1 cm to the capsule radius to do not be too close of the wall when doing the parallel SweepCast.
            FVector impactPointWithCapsule = forwardHit.ImpactPoint + forwardHit.ImpactNormal * (collisionShape.GetCapsuleRadius() + 1.0f);

            // Use the horizontal vector parallel to the wall to detect another wall at the left or the right of the character.
            // It works here because the floor is flat, but we may want to use a vector that is both parallel with the forward wall AND the ground where the character stands.
            TArray<FHitResult> parallelHitSide1;
            auto isParallelHitSide1 = SDTUtils::SweepCast(world, impactPointWithCapsule, parallelHitDirection, SidesWallRayCastDist, collisionShape, parallelHitSide1);
            TArray<FHitResult> parallelHitSide2;
            auto isParallelHitSide2 = SDTUtils::SweepCast(world, impactPointWithCapsule, -parallelHitDirection, SidesWallRayCastDist, collisionShape, parallelHitSide2);

            auto productForwardAndNormal = parallelHitDirection.Dot(forwardVector);
            if (!parallelHitSide1.IsEmpty() && !parallelHitSide2.IsEmpty())
            {
                // Turn to the direction where there is more space between the walls.
                if (parallelHitSide1[0].Distance > parallelHitSide2[0].Distance)
                {
                    rotationDirection = 1;
                    GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Walls on both sides. Direction is ") + FString::FromInt(rotationDirection));
                }
                else
                {
                    rotationDirection = -1;
                    GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Walls on both sides. Direction is ") + FString::FromInt(rotationDirection));
                }
            }
            else if (isParallelHitSide1)
            {
                rotationDirection = -1;
                GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Wall on right side only. Direction is ") + FString::FromInt(rotationDirection));
            }
            else if (isParallelHitSide2)
            {
                rotationDirection = 1;
                GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Wall on left side only. Direction is ") + FString::FromInt(rotationDirection));
            }
            // If the actor approximately faces the wall, choose a random direction.
            else if (-0.05f < productForwardAndNormal && productForwardAndNormal < 0.05f)
            {
                rotationDirection = FMath::RandBool() ? 1 : -1;
                GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Random Direction Chosen is ") + FString::FromInt(rotationDirection));
            }
            else
            {
                rotationDirection = FMath::Sign(productForwardAndNormal);
                GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Direction is ") + FString::FromInt(rotationDirection));
            }

            DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule + forwardVector * ForwardWallRayCastDist, FColor::Red, false, collisionDurationDebug, 0, isForwardHit ? 5.0f : 0.0f);
            DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule + parallelHitDirection * SidesWallRayCastDist, FColor::Red, false, collisionDurationDebug, 0, isParallelHitSide1 ? 5.0f : 0.0f);
            DrawDebugLine(world, impactPointWithCapsule, impactPointWithCapsule - parallelHitDirection * SidesWallRayCastDist, FColor::Red, false, collisionDurationDebug, 0, isParallelHitSide2 ? 5.0f : 0.0f);
            DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + forwardHit.ImpactNormal * 100.0), 5.0f, FColor::Green, false, collisionDurationDebug, 0, 2.0f);
            DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + upVector * 100.0), 5.0f, FColor::Blue, false, collisionDurationDebug, 0, 2.0f);
            DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + parallelHitDirection * 100.0), 5.0f, FColor::Magenta, false, collisionDurationDebug, 0, 2.0f);
            DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint - forwardVector * 100.0, forwardHit.ImpactPoint, 5.0f, FColor::Magenta, false, collisionDurationDebug, 0, 2.0f);
            UPrimitiveComponent *hitComponent = forwardHit.GetComponent();
            DrawDebugBox(world, hitComponent->Bounds.Origin, hitComponent->Bounds.BoxExtent, FColor::Green, false, 30.0f);
        }
    }
}

void ASDTAIController::Move(float deltaTime)
{
    auto character = GetCharacter();
    FVector forward = character->GetActorForwardVector();
    if (lastImpactNormal != FVector::ZeroVector)
    {
        // Calculate the next direction to go when turning to avoid a wall.
        FVector nextDirection = FRotator(0, rotationDirection * RotationAngleBySecond * deltaTime, 0).RotateVector(forward);
        if (nextDirection.Dot(lastImpactNormal) > 0 && !isForwardHit)
        {
            isTurningAround = false;
            GEngine->AddOnScreenDebugMessage(INDEX_NONE, 1.0, FColor::Green, FString("Actor finished rotation. Normal = ") + lastImpactNormal.ToString() + FString(", Forward = ") + forward.ToString());

            // Do a final adjustment to the direction to be exactly parallel with the wall.
            nextDirection = rotationDirection * lastImpactNormal.Cross(character->GetActorUpVector());

            lastImpactNormal = FVector::ZeroVector;
        }
        CalculateFarForwardTarget(nextDirection);
    }

    // debug print
    DrawDebugPoint(GetWorld(), targetMoveTo, 10.0f, FColor::Red);
    GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Yellow, FString::Printf(TEXT("[%s] Velocity: %f cm/s"), *character->GetName(), character->GetVelocity().Size()));
}

void ASDTAIController::SpeedControl(float deltaTime)
{
    auto chara = GetCharacter();
    auto currentSpeed = chara->GetVelocity().Size();
    // While the character is turning, reduce the speed below a maximum speed value if it is running too fast, or just keep its speed constant if below that maximum speed.
    float accelerationToApply;
    if (isTurningAround)
    {
        if (forwardImpactDistance <= 100.0f)
        {
            accelerationToApply = - 2 * Acceleration;
        }
        else if (chara->GetVelocity().Size() > 200)
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

    auto targetSpeed = FMath::Clamp(currentSpeed + accelerationToApply * deltaTime, 0.01f, MaxSpeed);
    chara->GetCharacterMovement()->MaxWalkSpeed = targetSpeed;
}



void ASDTAIController::DetectCollectible()
{
    auto pawn = GetPawn();
    UWorld *world = GetWorld();
    float radius = VisionDistance;
    bool drawDebug = true;
    // FHitResult hitResult;
    FCollisionObjectQueryParams objectQueryParams;
    FCollisionShape visionBox = FCollisionShape().MakeBox(FVector(1, 100, 100));
    FCollisionShape neighSphere = FCollisionShape().MakeSphere(radius);
    objectQueryParams.AddObjectTypesToQuery(ECC_GameTraceChannel5);

    TArray<struct FOverlapResult> outOverlaps;
    bool sthDetected = world->OverlapMultiByObjectType(outOverlaps, pawn->GetActorLocation(), FQuat::Identity, objectQueryParams, neighSphere);
    DrawDebugSphere(world, pawn->GetActorLocation(), radius, 24, FColor::Green);
    DrawDebugCone(world, pawn->GetActorLocation(), pawn->GetActorForwardVector(), VisionDistance, VisionAngle, VisionAngle, 24, FColor::Yellow);

    // world->LineTraceMultiByObjectType(hitResults, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, objectQueryParams);
    // bool sthDetected = world->LineTraceSingleByObjectType(hitResult, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, objectQueryParams);
    // bool sthDetected = world->SweepSingleByObjectType(hitResult, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, FQuat(0,0,0,0) ,objectQueryParams,visionBox);

    GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("detected=%d"), sthDetected));
    if (sthDetected)
    {
        for (int i = 0; i < outOverlaps.Num(); i++)
        {
            bool isVisible = IsInVisionCone(world, pawn, outOverlaps[i].GetActor());
            if (isVisible)
            {
                FVector vectPawnToTarget = outOverlaps[i].GetActor()->GetActorLocation() - pawn->GetActorLocation();
                float rotAngle = acos(FVector::DotProduct(pawn->GetActorForwardVector(), vectPawnToTarget) / vectPawnToTarget.Size()) * 180 / PI;
                FVector cross_prod = FVector::CrossProduct(vectPawnToTarget, pawn->GetActorForwardVector());
                float a = FVector::DotProduct(cross_prod, FVector::UpVector);
                if (a > 0)
                {
                    rotAngle = -rotAngle;
                }

                pawn->AddActorWorldRotation(FRotator(0, rotAngle, 0));

                // Debug drawing
                DrawDebugLine(world, pawn->GetActorLocation(), outOverlaps[i].GetActor()->GetActorLocation(), FColor::Blue, false, -1, 0, 5);
                DrawDebugBox(world, outOverlaps[i].GetActor()->GetActorLocation(), visionBox.GetBox(), FColor::Blue, false, -1, 0, 5);
                DrawDebugBox(world, pawn->GetActorLocation(), visionBox.GetBox(), FColor::Blue, false, -1, 0, 5);
                GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("You hit: %s"), *FString(outOverlaps[i].GetActor()->GetActorLabel())));
                GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("Angle to hit: %f"), rotAngle));
                break;
            }
        }
    }
}

bool ASDTAIController::IsInVisionCone(UWorld *world, AActor *pawn, AActor *targetActor)
{

    // We check if the target actor is too far from the pawn
    if (FVector::Dist2D(pawn->GetActorLocation(), targetActor->GetActorLocation()) > VisionDistance)
    {
        return false;
    }

    float collisionRadius;
    float collisionHalfHeight;
    pawn->GetSimpleCollisionCylinder(collisionRadius, collisionHalfHeight);
    FCollisionShape collisionCylinder = FCollisionShape().MakeCapsule(FVector(collisionRadius, collisionRadius, collisionHalfHeight));

    FCollisionObjectQueryParams objectQueryParams;
    objectQueryParams.AddObjectTypesToQuery(ECC_PhysicsBody);
    objectQueryParams.AddObjectTypesToQuery(ECC_WorldStatic);
    objectQueryParams.AddObjectTypesToQuery(ECC_WorldDynamic);
    objectQueryParams.AddObjectTypesToQuery(ECC_GameTraceChannel3);
    FHitResult hitResult;

    // We see if there is an obstacle
    bool sthDetected = world->SweepSingleByObjectType(hitResult, pawn->GetActorLocation(), targetActor->GetActorLocation(), FQuat(0, 0, 0, 0), objectQueryParams, collisionCylinder);
    if (sthDetected)
    {
        return false;
    }

    FVector direction = targetActor->GetActorLocation() - pawn->GetActorLocation();
    float value = FVector::DotProduct(direction.GetSafeNormal(), pawn->GetActorForwardVector().GetSafeNormal());
    auto angle = FMath::Acos(value);

    // We check if the target actor is in the cone
    if (FMath::Abs(angle) <= VisionAngle)
    {
        return true;
    }

    return false;
}
