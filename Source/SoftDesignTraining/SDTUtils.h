// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#define COLLISION_DEATH_OBJECT		ECollisionChannel::ECC_GameTraceChannel3
#define COLLISION_PLAYER        	ECollisionChannel::ECC_GameTraceChannel4
#define COLLISION_COLLECTIBLE     	ECollisionChannel::ECC_GameTraceChannel5

class SOFTDESIGNTRAINING_API SDTUtils
{
public:
    static bool Raycast(UWorld* uWorld, FVector sourcePoint, FVector targetPoint);
    static bool Raycast(UWorld* uWorld, FVector sourcePoint, FVector targetPoint, FHitResult& hitData);
    static bool SweepCast(UWorld* uWorld, FVector startPoint, FVector direction, float distance, const FCollisionShape& collisionShape, TArray<FHitResult>& hitData);
    static bool IsPlayerPoweredUp(UWorld* uWorld);
};
