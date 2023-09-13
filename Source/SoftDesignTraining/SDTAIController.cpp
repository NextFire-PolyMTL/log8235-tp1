// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTAIController.h"
#include "SoftDesignTraining.h"
#include "SDTUtils.h"

const float collisionDurationDebug = 15.0f;

void ASDTAIController::BeginPlay()
{
    Super::BeginPlay();
    auto chara = GetCharacter();
    auto moveComp = chara->GetCharacterMovement();
    moveComp->MaxWalkSpeed = MaxSpeed;
    moveComp->MaxAcceleration = Acceleration;
}

void ASDTAIController::Tick(float deltaTime)
{
    Super::Tick(deltaTime);
    DetectWalls();
    DetectCollectible();
    SpeedControl(deltaTime);
    Move(deltaTime);
}

void ASDTAIController::DetectWalls()
{
    auto world = GetWorld();
    auto character = GetCharacter();

    auto forwardVector = character->GetActorForwardVector();
    auto loc = character->GetActorLocation();
    // auto characterCollisionShape = character->GetCapsuleComponent()->GetCollisionShape();

    FHitResult forwardHit;
    isForwardHit = SDTUtils::Raycast(world, loc, loc + forwardVector * ForwardWallRayCastDist, forwardHit);
    // isForwardHit = SDTUtils::SweepCast(world, character, forwardVector, ForwardWallRayCastDist, forwardHit);

    if (isForwardHit)
    {
        if (!isTurningAround)
        {
            // Take only one decision on the side to turn around and stick on this rotation side until no
            // forward collision is detected. This is to avoid to stay stuck in a corner.
            isTurningAround = true;

            // Get a rotator with yaw and pitch angle only from the normal vector.
            auto rotatorInPlane = forwardHit.ImpactNormal.ToOrientationRotator();
            // Take the up vector and rotate it to put it inside the plane of the wall.
            auto upVectorInPlane = rotatorInPlane.RotateVector(FVector::UpVector);
            // Calculate the cross product to get a horizontal vector on the plane of the wall pointing to the right.
            auto horizontalDirectionInPlane = forwardHit.ImpactNormal.Cross(upVectorInPlane);

            // Use the horizontal vector parallel to the wall to detect another wall at the left or the right of the character.
            // It works here because the floor is flat, but we may want to use a vector that is both parallel with the forward wall AND the ground where the character stands.
            FHitResult parallelHitSide1;
            auto isParallelHitSide1 = SDTUtils::Raycast(world, loc, loc + horizontalDirectionInPlane * SidesWallRayCastDist, parallelHitSide1);
            FHitResult parallelHitSide2;
            auto isParallelHitSide2 = SDTUtils::Raycast(world, loc, loc - horizontalDirectionInPlane * SidesWallRayCastDist, parallelHitSide2);

            auto productForwardAndNormal = horizontalDirectionInPlane.Dot(forwardVector);
            if (isParallelHitSide1 && isParallelHitSide2)
            {
                // Turn to the direction where there is more space between the walls.
                if (parallelHitSide1.Distance > parallelHitSide2.Distance)
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

            DrawDebugLine(world, loc, loc + forwardVector * ForwardWallRayCastDist, FColor::Red, false, collisionDurationDebug, 0, isForwardHit ? 5.0f : 0.0f);
            DrawDebugLine(world, loc, loc + horizontalDirectionInPlane * SidesWallRayCastDist, FColor::Red, false, collisionDurationDebug, 0, isParallelHitSide1 ? 5.0f : 0.0f);
            DrawDebugLine(world, loc, loc - horizontalDirectionInPlane * SidesWallRayCastDist, FColor::Red, false, collisionDurationDebug, 0, isParallelHitSide2 ? 5.0f : 0.0f);
            DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + forwardHit.ImpactNormal * 100.0), 5.0f, FColor::Green, false, collisionDurationDebug, 0, 2.0f);
            DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + upVectorInPlane * 100.0), 5.0f, FColor::Blue, false, collisionDurationDebug, 0, 2.0f);
            DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint, (forwardHit.ImpactPoint + horizontalDirectionInPlane * 100.0), 5.0f, FColor::Magenta, false, collisionDurationDebug, 0, 2.0f);
            DrawDebugDirectionalArrow(world, forwardHit.ImpactPoint - forwardVector * 100.0, forwardHit.ImpactPoint, 5.0f, FColor::Magenta, false, collisionDurationDebug, 0, 2.0f);
            UPrimitiveComponent *hitComponent = forwardHit.GetComponent();
            DrawDebugBox(world, hitComponent->Bounds.Origin, hitComponent->Bounds.BoxExtent, FColor::Green, false, 30.0f);
        }

        lastImpactNormal = forwardHit.ImpactNormal;
    }
}

void ASDTAIController::Move(float deltaTime)
{
    auto character = GetCharacter();
    FVector forward = character->GetActorForwardVector();
    if (lastImpactNormal != FVector::ZeroVector)
    {
        // rotate
        character->AddActorWorldRotation(FRotator(0, rotationDirection * RotationAngleBySecond * deltaTime, 0));
        forward = character->GetActorForwardVector();
        if (forward.Dot(lastImpactNormal) > 0 && !isForwardHit)
        {
            isTurningAround = false;
            GEngine->AddOnScreenDebugMessage(INDEX_NONE, 1.0, FColor::Green, FString("Actor finished rotation. Normal = ") + lastImpactNormal.ToString() + FString(", Forward = ") + forward.ToString());

            // The actor is probably not exactly parallel with the wall. We may want to do an adjustment of the actor rotation to
            // be exactly parallel with the last impact.
            lastImpactNormal = FVector::ZeroVector;
        }
    }

    // move forward
    character->AddMovementInput(forward);
    // debug print
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
        if (chara->GetVelocity().Size() > 200)
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

    auto targetSpeed = FMath::Min(currentSpeed + accelerationToApply * deltaTime, MaxSpeed);
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
