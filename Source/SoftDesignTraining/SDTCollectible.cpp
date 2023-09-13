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
}

void ASDTCollectible::Collect()
{
    GetWorld()->GetTimerManager().SetTimer(m_CollectCooldownTimer, this, &ASDTCollectible::OnCooldownDone, m_CollectCooldownDuration, false);

    GetStaticMeshComponent()->SetVisibility(false);
}

void ASDTCollectible::OnCooldownDone()
{
    GetWorld()->GetTimerManager().ClearTimer(m_CollectCooldownTimer);

    GetStaticMeshComponent()->SetVisibility(true);
}

bool ASDTCollectible::IsOnCooldown()
{
    return m_CollectCooldownTimer.IsValid();
}

void ASDTCollectible::Tick(float deltaTime)
{
    Super::Tick(deltaTime);

    if (isMoveable)
    {
        Move();
    }
}

void ASDTCollectible::Move()
{
    // Calcul de la nouvelle position en utilisant la méthode d'Euler
    FVector NewLocation = GetActorLocation() + (TargetDir * CurrentSpeed * GetWorld()->DeltaTimeSeconds);

    // Augmentation de la vitesse en fonction de l'accélération
    CurrentSpeed += Acceleration * GetWorld()->DeltaTimeSeconds;

    // Limiter la vitesse maximale
    CurrentSpeed = FMath::Min(CurrentSpeed, MaxSpeed);

    // Mettre à jour la position
    SetActorLocation(NewLocation);

    GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Green, FString::Printf(TEXT("[%s] Location: %s"), *GetName(), *NewLocation.ToString()));

}
