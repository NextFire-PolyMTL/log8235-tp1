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
    float radius = VisionDistance;
    bool drawDebug = true;
    //FHitResult hitResult;
    FCollisionObjectQueryParams objectQueryParams;
    FCollisionShape visionBox= FCollisionShape().MakeBox(FVector(1, 100, 100));
    FCollisionShape neighSphere = FCollisionShape().MakeSphere(radius);
    objectQueryParams.AddObjectTypesToQuery(ECC_GameTraceChannel5);
    
   

    TArray<struct FOverlapResult> outOverlaps;
    bool sthDetected=world->OverlapMultiByObjectType(outOverlaps, pawn->GetActorLocation(), FQuat::Identity, objectQueryParams, neighSphere);
    DrawDebugSphere(world,pawn->GetActorLocation(), radius, 24, FColor::Green);
    DrawDebugCone(world, pawn->GetActorLocation(), pawn->GetActorForwardVector(), VisionDistance, VisionAngle, VisionAngle, 24, FColor::Yellow);
    
    
    //world->LineTraceMultiByObjectType(hitResults, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, objectQueryParams);
    //bool sthDetected = world->LineTraceSingleByObjectType(hitResult, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, objectQueryParams);
    //bool sthDetected = world->SweepSingleByObjectType(hitResult, pawn->GetActorLocation(), pawn->GetActorLocation() + pawn->GetActorForwardVector() * 1000, FQuat(0,0,0,0) ,objectQueryParams,visionBox);
    

    GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("detected=%d"), sthDetected));
    if (sthDetected) {
        for (int i = 0;i < outOverlaps.Num();i++) {
            bool isVisible = IsInVisionCone(world,pawn,outOverlaps[i].GetActor());
            if (isVisible) {
                FVector vectPawnToTarget = outOverlaps[i].GetActor()->GetActorLocation() - pawn->GetActorLocation();
                float rotAngle = acos(FVector::DotProduct(pawn->GetActorForwardVector(), vectPawnToTarget) / vectPawnToTarget.Size()) * 180 / PI;
                FVector cross_prod = FVector::CrossProduct(vectPawnToTarget, pawn->GetActorForwardVector());
                float a = FVector::DotProduct(cross_prod, FVector::UpVector);
                if (a > 0) {
                    rotAngle = -rotAngle;
                }

                pawn->AddActorWorldRotation(FRotator(0, rotAngle, 0));

                //Debug drawing
                DrawDebugLine(world, pawn->GetActorLocation(), outOverlaps[i].GetActor()->GetActorLocation(), FColor::Blue, false, -1, 0, 5);
                DrawDebugBox(world, outOverlaps[i].GetActor()->GetActorLocation(), visionBox.GetBox(), FColor::Blue, false, -1, 0, 5);
                DrawDebugBox(world, pawn->GetActorLocation(), visionBox.GetBox(), FColor::Blue, false, -1, 0, 5);
                GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("You hit: %s"), *FString(outOverlaps[i].GetActor()->GetActorLabel())));
                GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Blue, FString::Printf(TEXT("Angle to hit: %f"), rotAngle));
                break;
            }
        }
    }

}
bool ASDTAIController::IsInVisionCone(UWorld* world, AActor* pawn, AActor* targetActor) {

    //We check if the target actor is too far from the pawn
    if (FVector::Dist2D(pawn->GetActorLocation(), targetActor->GetActorLocation()) > VisionDistance)
    {
        return false;
    }

    float collisionRadius;
    float collisionHalfHeight;
    pawn->GetSimpleCollisionCylinder(collisionRadius, collisionHalfHeight);
    FCollisionShape collisionCylinder = FCollisionShape().MakeCapsule(FVector(collisionRadius, collisionRadius, collisionHalfHeight));

    FCollisionObjectQueryParams objectQueryParams;
    objectQueryParams.AddObjectTypesToQuery(ECC_PhysicsBody);
    objectQueryParams.AddObjectTypesToQuery(ECC_WorldStatic);
    objectQueryParams.AddObjectTypesToQuery(ECC_WorldDynamic);
    objectQueryParams.AddObjectTypesToQuery(ECC_GameTraceChannel3);
    FHitResult hitResult;

    //We see if there is an obstacle
    bool sthDetected = world->SweepSingleByObjectType(hitResult, pawn->GetActorLocation(), targetActor->GetActorLocation(), FQuat(0, 0, 0, 0), objectQueryParams, collisionCylinder);
    if (sthDetected) {
        return false;
    }

    FVector direction = targetActor->GetActorLocation() - pawn->GetActorLocation();
    float value = FVector::DotProduct(direction.GetSafeNormal(), pawn->GetActorForwardVector().GetSafeNormal());
    auto angle = FMath::Acos(value);

    //We check if the target actor is in the cone
    if (FMath::Abs(angle) <= VisionAngle) {
        return true;
    }

    return false;

}
