// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"

#include "SDTAIController.generated.h"

/**
 *
 */
UCLASS(ClassGroup = AI, config = Game)
class SOFTDESIGNTRAINING_API ASDTAIController : public AAIController
{
    GENERATED_BODY()
public:
    virtual void BeginPlay() override;
    virtual void Tick(float deltaTime) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float VisionDistance = 500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float VisionAngle = PI / 3.0f;;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float MaxSpeed = 500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Acceleration = 250.0f;

private:
    void Move();
    void DetectCollectible();
    bool IsInVisionCone(UWorld* world, AActor* pawn, AActor* targetActor);

    FVector TargetDir = FVector::LeftVector; // placeholder
};
