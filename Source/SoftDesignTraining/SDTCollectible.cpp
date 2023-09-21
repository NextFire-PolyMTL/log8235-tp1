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

    FVector Origin;
    FVector BoxExtent;

    GetActorBounds(false, Origin, BoxExtent);

    // Calcul de la nouvelle position en utilisant la méthode d'Euler
    FVector NewLocation = Origin + (TargetDir * CurrentSpeed * GetWorld()->DeltaTimeSeconds);

    UWorld* World = GetWorld();
    FQuat Rotation = FQuat::Identity;
    float TimeToStop = CurrentSpeed / Acceleration;
    FVector SphereCenter = Origin + (TargetDir * CurrentSpeed * TimeToStop);
    float SphereRadius = BoxExtent.X;
    TArray<FOverlapResult> OutOverlaps;

    // Détection d'objets dans la sphère
    bool bHit = World->OverlapMultiByObjectType(OutOverlaps, SphereCenter, Rotation, ECC_WorldStatic, FCollisionShape::MakeSphere(SphereRadius));

    //DrawDebugSphere(GetWorld(), SphereCenter, SphereRadius, 32, FColor::Green, false, 0.1f);

    if (bHit)
    {
        // Réduction de la vitesse en utilisant l'accélération négative
        CurrentSpeed -= Acceleration * GetWorld()->DeltaTimeSeconds;

        CurrentSpeed = FMath::Max(CurrentSpeed, 0);

        // Si la vitesse est inférieure ou égale à zéro, inversez la direction
        if (CurrentSpeed <= 100.0f)
        {
            CurrentSpeed = 0.0f;
            TargetDir = -TargetDir; // Inverser la direction
        }
    }
    else
    {
        // Augmentation de la vitesse en fonction de l'accélération
        CurrentSpeed += Acceleration * GetWorld()->DeltaTimeSeconds;

        // Limiter la vitesse maximale
        CurrentSpeed = FMath::Min(CurrentSpeed, MaxSpeed);
    }

    //GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Green, FString::Printf(TEXT("Vitesse : %f m/s"), CurrentSpeed));
  
    // Mettre à jour la position
    SetActorLocation(NewLocation);
    
}
