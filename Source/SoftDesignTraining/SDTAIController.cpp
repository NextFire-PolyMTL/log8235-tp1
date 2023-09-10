// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTAIController.h"
#include "SoftDesignTraining.h"
#include "SDTUtils.h"

const float collisionDurationDebug = 15.0f;

void ASDTAIController::BeginPlay()
{
    Super::BeginPlay();
    auto chara = GetCharacter();
    auto *moveComp = chara->GetCharacterMovement();
    moveComp->MaxWalkSpeed = MaxSpeed;
    moveComp->MaxAcceleration = Acceleration;
}

void ASDTAIController::Tick(float deltaTime)
{
    Super::Tick(deltaTime);
    DetectWalls();
    Move(deltaTime);
}

void ASDTAIController::DetectWalls()
{
    auto world = GetWorld();
    auto character = GetCharacter();

    auto forwardVector = character->GetActorForwardVector();
    auto loc = character->GetActorLocation();
    //auto characterCollisionShape = character->GetCapsuleComponent()->GetCollisionShape();

    FHitResult forwardHit;
    isForwardHit = SDTUtils::Raycast(world, loc, loc + forwardVector * ForwardWallRayCastDist, forwardHit);
    //isForwardHit = SDTUtils::SweepCast(world, character, forwardVector, ForwardWallRayCastDist, forwardHit);

    if (isForwardHit)
    {
        if (!isTurningAround) {
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
            if (isParallelHitSide1 && isParallelHitSide2) {
                // Turn to the direction where there is more space between the walls.
                if (parallelHitSide1.Distance > parallelHitSide2.Distance) {
                    rotationDirection = 1;
                    GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Walls on both sides. Direction is ") + FString::FromInt(rotationDirection));
                }
                else {
                    rotationDirection = -1;
                    GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Walls on both sides. Direction is ") + FString::FromInt(rotationDirection));
                }
            }
            else if (isParallelHitSide1) {
                rotationDirection = -1;
                GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Wall on right side only. Direction is ") + FString::FromInt(rotationDirection));
            }
            else if (isParallelHitSide2) {
                rotationDirection = 1;
                GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Wall on left side only. Direction is ") + FString::FromInt(rotationDirection));
            }
            // If the actor approximately faces the wall, choose a random direction.
            else if (-0.05f < productForwardAndNormal && productForwardAndNormal < 0.05f) {
                rotationDirection = FMath::RandBool() ? 1 : -1;
                GEngine->AddOnScreenDebugMessage(INDEX_NONE, 5.0, FColor::Blue, FString("Random Direction Chosen is ") + FString::FromInt(rotationDirection));
            }
            else {
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
            UPrimitiveComponent* hitComponent = forwardHit.GetComponent();
            DrawDebugBox(world, hitComponent->Bounds.Origin, hitComponent->Bounds.BoxExtent, FColor::Green, false, 30.0f);
        }

        lastImpactNormal = forwardHit.ImpactNormal;
    }
}

void ASDTAIController::Move(float deltaTime)
{
    auto character = GetCharacter();
    FVector forward;
    if (lastImpactNormal != FVector::ZeroVector) {
        // While the character is turning, reduce the speed below a maximum speed value if it is running too fast, or just keep its speed constant if below that maximum speed.
        // TODO: This does not work.
        if (character->GetVelocity().Size() > 200) {
            currentSpeed = FMath::Min(-20 * Acceleration * deltaTime + character->GetVelocity().Size(), MaxSpeed);
        }
        else {
            currentSpeed = character->GetVelocity().Size();
        }

        // rotate
        character->AddActorWorldRotation(FRotator(0, rotationDirection * RotationAngleBySecond * deltaTime, 0));
        forward = character->GetActorForwardVector();
        if (forward.Dot(lastImpactNormal) > 0 && !isForwardHit) {
            isTurningAround = false;
            GEngine->AddOnScreenDebugMessage(INDEX_NONE, 1.0, FColor::Green, FString("Actor finished rotation. Normal = ") + lastImpactNormal.ToString() + FString(", Forward = ") + forward.ToString());

            // The actor is probably not exactly parallel with the wall. We may want to do an adjustment of the actor rotation to
            // be exactly parallel with the last impact.
            lastImpactNormal = FVector::ZeroVector;
        }
    }
    else {
        currentSpeed = FMath::Min(Acceleration * deltaTime + character->GetVelocity().Size(), MaxSpeed);
        forward = character->GetActorForwardVector();
    }


    // move forward
    character->AddMovementInput(forward, currentSpeed * deltaTime);
    // debug print
    GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Yellow, FString::Printf(TEXT("[%s] Velocity: %f cm/s"), *character->GetName(), character->GetVelocity().Size()));
}
