// Copyright 1998-2015 Epic Games, Inc. All Rights Reserved.

#include "SoftDesignTrainingCharacter.h"
#include "SoftDesignTraining.h"
#include "SoftDesignTrainingMainCharacter.h"
#include "SDTUtils.h"
#include "DrawDebugHelpers.h"
#include "SDTCollectible.h"

ASoftDesignTrainingCharacter::ASoftDesignTrainingCharacter()
{
    GetCapsuleComponent()->InitCapsuleSize(42.f, 96.0f);
}

void ASoftDesignTrainingCharacter::BeginPlay()
{
    Super::BeginPlay();

    GetCapsuleComponent()->OnComponentBeginOverlap.AddDynamic(this, &ASoftDesignTrainingCharacter::OnBeginOverlap);
    m_StartingPosition = GetActorLocation();
}

void ASoftDesignTrainingCharacter::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    GEngine->RemoveOnScreenDebugMessage((uint64)GetUniqueID());
}

void ASoftDesignTrainingCharacter::Tick(float deltaTime)
{
    Super::Tick(deltaTime);

    auto time = GetWorld()->GetTimeSeconds();
    auto timeString = FString::Printf(TEXT("%.1fs"), time);
    auto pickupString = FString::Printf(TEXT("%d pickups"), m_PickupCount);
    auto deathString = FString::Printf(TEXT("%d deaths"), m_DeathCount);

    GEngine->AddOnScreenDebugMessage((uint64)GetUniqueID(), INFINITY, FColor::White, FString::Printf(TEXT("[%s] %.1fs, %d pickups, %d deaths"), *GetName(), time, m_PickupCount, m_DeathCount));

    auto world = GetWorld();
    auto loc = GetActorLocation();
    DrawDebugString(world, loc - FVector::UpVector * 100, timeString, nullptr, FColor::White, 0.0f, true);
    DrawDebugString(world, loc - FVector::UpVector * 200, pickupString, nullptr, FColor::White, 0.0f, true);
    DrawDebugString(world, loc - FVector::UpVector * 300, deathString, nullptr, FColor::White, 0.0f, true);
}

void ASoftDesignTrainingCharacter::OnBeginOverlap(UPrimitiveComponent *OverlappedComponent, AActor *OtherActor, UPrimitiveComponent *OtherComponent, int32 OtherBodyIndex, bool bFromSweep, const FHitResult &SweepResult)
{
    if (OtherComponent->GetCollisionObjectType() == COLLISION_DEATH_OBJECT)
    {
        OnDeath();
        SetActorLocation(m_StartingPosition);
    }
    else if (ASDTCollectible *collectibleActor = Cast<ASDTCollectible>(OtherActor))
    {
        if (!collectibleActor->IsOnCooldown())
        {
            OnCollectPowerUp();
        }

        collectibleActor->Collect();
    }
    else if (ASoftDesignTrainingMainCharacter *mainCharacter = Cast<ASoftDesignTrainingMainCharacter>(OtherActor))
    {
        if (mainCharacter->IsPoweredUp())
            OnDeath();
            SetActorLocation(m_StartingPosition);
    }
}

void ASoftDesignTrainingCharacter::OnCollectPowerUp()
{
    m_PickupCount++;
}

void ASoftDesignTrainingCharacter::OnDeath()
{
    m_DeathCount++;
}
