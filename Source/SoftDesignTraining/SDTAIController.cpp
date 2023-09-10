// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTAIController.h"
#include "SoftDesignTraining.h"

void ASDTAIController::BeginPlay()
{
    Super::BeginPlay();
    auto *chara = GetCharacter();
    auto *moveComp = chara->GetCharacterMovement();
    moveComp->MaxWalkSpeed = MaxSpeed;
    moveComp->MaxAcceleration = Acceleration;
}

void ASDTAIController::Tick(float deltaTime)
{
    Super::Tick(deltaTime);
    Move();
    DetectCollectible();
}

void ASDTAIController::Move()
{
    auto *pawn = GetPawn();
    // rotate
    auto rot = FQuat::FindBetween(pawn->GetActorForwardVector(), TargetDir);
    pawn->AddActorWorldRotation(rot);
    // move forward
    pawn->AddMovementInput(pawn->GetActorForwardVector());
    // debug print
    GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Yellow, FString::Printf(TEXT("[%s] Velocity: %f m/s"), *pawn->GetName(), pawn->GetVelocity().Size()));
}

void ASDTAIController::DetectCollectible()
{
    auto* pawn = GetPawn();
    UWorld* world = GetWorld();
    FHitResult hitResult;
    //TArray<struct FHitResult> hitResults;
    FCollisionObjectQueryParams objectQueryParams;// = FCollisionObjectQueryParams::DefaultObjectQueryParam;
    /*
    objectQueryParams.AddObjectTypesToQuery(ECC_PhysicsBody);
    objectQueryParams.AddObjectTypesToQuery(ECC_WorldStatic);
    objectQueryParams.AddObjectTypesToQuery(ECC_WorldDynamic);
    objectQueryParams.AddObjectTypesToQuery(ECC_Destructible);
    objectQueryParams.AddObjectTypesToQuery(ECC_EngineTraceChannel1);
    */

    objectQueryParams.AddObjectTypesToQuery(ECC_GameTraceChannel5);
    



    
    //world->LineTraceMultiByObjectType(hitResults, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, objectQueryParams);
    bool sthDetected = world->LineTraceSingleByObjectType(hitResult, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, objectQueryParams);
    GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("detected=%d"), sthDetected));
    //Draw hits
    if (sthDetected) {
        //GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("nb of hit: %d"), hitResults.Num()));
        GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("You hit: %s"), *FString(hitResult.GetActor()->GetName())));
        DrawDebugLine(world, pawn->GetActorLocation(), hitResult.GetActor()->GetActorLocation(), FColor::Blue, false, -1, 0, 5);
    }
    else {
        DrawDebugLine(world, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, FColor::Red, false, -1, 0, 5);
    }


}
