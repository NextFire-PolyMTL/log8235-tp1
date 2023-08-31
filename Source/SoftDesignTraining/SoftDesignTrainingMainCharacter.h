// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SoftDesignTrainingCharacter.h"
#include "SoftDesignTrainingMainCharacter.generated.h"

/**
 * 
 */
UCLASS()
class SOFTDESIGNTRAINING_API ASoftDesignTrainingMainCharacter : public ASoftDesignTrainingCharacter
{
    GENERATED_BODY()
public:
    ASoftDesignTrainingMainCharacter();

    virtual void OnCollectPowerUp() override;

    bool IsPoweredUp() { return m_IsPoweredUp; }

protected:

    virtual void OnBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, UPrimitiveComponent* OtherComponent, int32 OtherBodyIndex, bool bFromSweep, const FHitResult& SweepResult) override;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Camera, meta = (AllowPrivateAccess = "true"))
        class UCameraComponent* m_TopDownCameraComponent;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = Camera, meta = (AllowPrivateAccess = "true"))
        class USpringArmComponent* m_CameraBoom;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AI)
        float m_PowerUpDuration = 10.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AI)
        UMaterialInterface* m_PoweredUpMaterial;

    void OnPowerUpDone();

    bool m_IsPoweredUp;
    FTimerHandle m_PowerUpTimer;
};
