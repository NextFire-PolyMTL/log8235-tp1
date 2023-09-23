// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTCollectible.h"
#include "SoftDesignTraining.h"


ASDTCollectible::ASDTCollectible()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ASDTCollectible::BeginPlay()
{
    Super::BeginPlay();

    InitialPosition = GetActorLocation();
    ResetMovementComponents();
}

void ASDTCollectible::Collect()
{
    GetWorld()->GetTimerManager().SetTimer(CollectCooldownTimer, this, &ASDTCollectible::OnCooldownDone, CollectCooldownDuration, false);

    GetStaticMeshComponent()->SetVisibility(false);
}

void ASDTCollectible::OnCooldownDone()
{
    GetWorld()->GetTimerManager().ClearTimer(CollectCooldownTimer);

    GetStaticMeshComponent()->SetVisibility(true);
    ResetMovementComponents();
}

bool ASDTCollectible::IsOnCooldown()
{
    return CollectCooldownTimer.IsValid();
}

void ASDTCollectible::Tick(float deltaTime)
{
    Super::Tick(deltaTime);

    if (IsMoveable && !IsOnCooldown())
    {
        Move(deltaTime);
    }
}

void ASDTCollectible::ResetMovementComponents()
{
    CurrentSpeed = FVector(0, MaxSpeed, 0);
    CurrentAcceleration = FVector(0, Acceleration, 0);
    SetActorLocation(InitialPosition);
}

void ASDTCollectible::Move(float deltaTime)
{
    auto world = GetWorld();
    FVector origin;
    FVector boxExtent;

    GetActorBounds(false, origin, boxExtent);
    float sphereRadius = boxExtent.X;

    FVector currentDir = CurrentSpeed;
    currentDir.Normalize();

    // Do a wall detection only if the current acceleration vector is in the same direction
    // as the current speed vector.
    if (CurrentAcceleration.Dot(CurrentSpeed) > 0)
    {
        // Calculate the expected sphere position if it were to decelerate at a constant rate to zero speed.
        float TimeToStop = CurrentSpeed.Size() / CurrentAcceleration.Size();
        FVector spherePosAtZero = origin + (CurrentSpeed * TimeToStop) - CurrentAcceleration * FMath::Square(TimeToStop) / 2;
        // Add an additional distance from the wall to indicate at which distance the collectible should stop.
        spherePosAtZero += currentDir * DistanceFromWall;

        // Detect if there is a wall between the actual position and the expected position at zero speed.
        FHitResult hitResult;
        bool bHit = world->SweepSingleByObjectType(hitResult, origin, spherePosAtZero, FQuat::Identity, ECC_WorldStatic, FCollisionShape::MakeSphere(sphereRadius));
        if (bHit)
        {
            CurrentAcceleration = -CurrentAcceleration;
            CurrentHitNormal = hitResult.ImpactNormal;
            CurrentHitPoint = hitResult.ImpactPoint;
            DrawDebugSphere(world, origin - hitResult.Distance * hitResult.ImpactNormal, sphereRadius, 32, FColor::Green, false, TimeToStop);
        }
        else if (CurrentHitNormal != FVector::ZeroVector)
        {
            CurrentHitNormal = FVector::ZeroVector;
            CurrentHitPoint = FVector::ZeroVector;
        }

        DrawDebugSphere(GetWorld(), spherePosAtZero, sphereRadius, 32, FColor::Red);
    }

    // Use semi-implicit Euler to calculate the next speed and next position.
    CurrentSpeed = CurrentSpeed + CurrentAcceleration * deltaTime;
    // Restrict the CurrentSpeed to MaxSpeed.
    float speedSize = CurrentSpeed.Size();
    if (speedSize > MaxSpeed)
    {
        CurrentSpeed *= MaxSpeed / speedSize;
    }
    FVector newLocation = origin + CurrentSpeed * deltaTime;

    // Do an additional check to be sure to do not pass over the collision plane.
    auto spherePotentialHitPoint = origin + currentDir * sphereRadius;
    if (CurrentHitNormal != FVector::ZeroVector && CurrentHitNormal.Dot(spherePotentialHitPoint - CurrentHitPoint) < 0)
    {
        CurrentSpeed = FVector::ZeroVector;
        newLocation = CurrentHitPoint + CurrentHitNormal * sphereRadius;
    }

    // Update the position.
    SetActorLocation(newLocation);

    DrawDebugDirectionalArrow(world, spherePotentialHitPoint, spherePotentialHitPoint + CurrentSpeed, 20.0f, FColor::Blue, false, -1.0f, 0U, 5.0f);
}
