// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTUtils.h"
#include "SoftDesignTraining.h"
#include "SoftDesignTrainingMainCharacter.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"

/*static*/ bool SDTUtils::Raycast(UWorld *uWorld, FVector sourcePoint, FVector targetPoint)
{
    FHitResult hitData;
    return SDTUtils::Raycast(uWorld, sourcePoint, targetPoint, hitData);
}

bool SDTUtils::Raycast(UWorld *uWorld, FVector sourcePoint, FVector targetPoint, FHitResult &hitData)
{
    FCollisionQueryParams TraceParams(FName(TEXT("VictoreCore Trace")), true);

    return uWorld->LineTraceSingleByChannel(hitData, sourcePoint, targetPoint, ECC_Pawn, TraceParams);
}

bool SDTUtils::SweepCast(UWorld* uWorld, FVector startPoint, FVector direction, float distance, const FCollisionShape& collisionShape, TArray<FHitResult>& hitData)
{
    FCollisionObjectQueryParams queryParams(ECC_TO_BITFIELD(ECC_WorldStatic) | ECC_TO_BITFIELD(ECC_GameTraceChannel3));
    FCollisionQueryParams TraceParams(FName(TEXT("VictoreCore Trace")), true);

    return uWorld->SweepMultiByObjectType(hitData, startPoint, startPoint + direction * distance, FQuat::Identity, queryParams, collisionShape, TraceParams);
}

bool SDTUtils::IsPlayerPoweredUp(UWorld *uWorld)
{
    ACharacter *playerCharacter = UGameplayStatics::GetPlayerCharacter(uWorld, 0);
    if (!playerCharacter)
        return false;

    ASoftDesignTrainingMainCharacter *castedPlayerCharacter = Cast<ASoftDesignTrainingMainCharacter>(playerCharacter);
    if (!castedPlayerCharacter)
        return false;

    return castedPlayerCharacter->IsPoweredUp();
}
