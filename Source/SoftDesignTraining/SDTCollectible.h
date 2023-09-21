// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "Engine/StaticMeshActor.h"
#include "SDTCollectible.generated.h"



UCLASS()
class SOFTDESIGNTRAINING_API ASDTCollectible : public AStaticMeshActor
{
	GENERATED_BODY()
public:
    ASDTCollectible();

    void Collect();
    void OnCooldownDone();
    bool IsOnCooldown();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AI)
    float m_CollectCooldownDuration = 10.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AI)
        bool isMoveable = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AI)
        float MaxSpeed = 500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AI)
        float Acceleration = 250.0f;

    virtual void Tick(float deltaTime) override;
    virtual void BeginPlay() override;

    FVector initialPosition;

protected:
    float CurrentSpeed = 0.0f;
    FVector TargetDir = FVector::RightVector;

    void Move();

    FTimerHandle m_CollectCooldownTimer;
	
};
