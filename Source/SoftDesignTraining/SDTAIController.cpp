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
    /*
    // rotate
    auto rot = FQuat::FindBetween(pawn->GetActorForwardVector(), TargetDir);
    pawn->AddActorWorldRotation(rot);
    */
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
    FCollisionObjectQueryParams objectQueryParams;
    FCollisionShape visionBox= FCollisionShape().MakeBox(FVector(1, 100, 100));
    /*
    objectQueryParams.AddObjectTypesToQuery(ECC_PhysicsBody);
    objectQueryParams.AddObjectTypesToQuery(ECC_WorldStatic);
    objectQueryParams.AddObjectTypesToQuery(ECC_WorldDynamic);
    */
    

    objectQueryParams.AddObjectTypesToQuery(ECC_GameTraceChannel5);
    



    
    //world->LineTraceMultiByObjectType(hitResults, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, objectQueryParams);
    //bool sthDetected = world->LineTraceSingleByObjectType(hitResult, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, objectQueryParams);
    bool sthDetected = world->SweepSingleByObjectType(hitResult, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, FQuat(0,0,0,0) ,objectQueryParams,visionBox);
    

    GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("detected=%d"), sthDetected));
    if (sthDetected) {

        
        FVector vectPawnToTarget = hitResult.GetActor()->GetActorLocation() - pawn->GetActorLocation();
        float rotAngle = acos(FVector::DotProduct(pawn->GetActorForwardVector(),vectPawnToTarget)/ vectPawnToTarget.Size())*180/PI;
        FVector cross_prod = FVector::CrossProduct(vectPawnToTarget, pawn->GetActorForwardVector());
        float a = FVector::DotProduct(cross_prod, FVector::UpVector);
        if (a > 0) {
            rotAngle = -rotAngle;
        }
        
        pawn->AddActorWorldRotation(FRotator(0, rotAngle, 0));

        //Debug drawing
        DrawDebugLine(world, pawn->GetActorLocation(), hitResult.GetActor()->GetActorLocation(), FColor::Blue, false, -1, 0, 5);
        DrawDebugBox(world, hitResult.GetActor()->GetActorLocation(), visionBox.GetBox(), FColor::Blue, false, -1, 0, 5);
        DrawDebugBox(world, pawn->GetActorLocation(), visionBox.GetBox(), FColor::Blue, false, -1, 0, 5);
        GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("You hit: %s"), *FString(hitResult.GetActor()->GetActorLabel())));
        GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("Angle to hit: %f"), rotAngle));
    }
    else {

        //Debug Drawing
        DrawDebugLine(world, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, FColor::Red, false, -1, 0, 5);
        DrawDebugBox(world, pawn->GetActorLocation(), visionBox.GetBox(), FColor::Red, false, -1, 0, 5);
    }


}
