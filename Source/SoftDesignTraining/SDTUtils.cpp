// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTUtils.h"
#include "SoftDesignTraining.h"
#include "SoftDesignTrainingMainCharacter.h"
#include "DrawDebugHelpers.h"
#include "Components/CapsuleComponent.h"
#include "Engine/World.h"

#include <iostream>

bool GetAvoidingLeftPoint(UWorld* world, const FHitResult& obstacle, FVector start, UCapsuleComponent* actorCapsule, FVector& avoidingPoint)
{
    float capsuleRadius = actorCapsule->GetScaledCapsuleRadius() + 1;
    FBox box = obstacle.GetActor()->GetComponentsBoundingBox();
    float minX = box.Min.X;
    float maxX = box.Max.X;
    float minY = box.Min.Y;
    float maxY = box.Max.Y;
    if (start.X <= maxX && start.Y < minY)
    {
        avoidingPoint = FVector(maxX + capsuleRadius, minY - capsuleRadius, start.Z);
    }
    else if (start.X > maxX && start.Y <= maxY)
    {
        avoidingPoint = FVector(maxX + capsuleRadius, maxY + capsuleRadius, start.Z);
    }
    else if (start.X >= minX && start.Y > maxY)
    {
        avoidingPoint = FVector(minX - capsuleRadius, maxY + capsuleRadius, start.Z);
    }
    else
    {
        avoidingPoint = FVector(minX - capsuleRadius, minY - capsuleRadius, start.Z);
    }

    return !world->OverlapBlockingTestByChannel(avoidingPoint, FQuat::Identity, ECollisionChannel::ECC_Pawn, actorCapsule->GetCollisionShape());
}

bool GetAvoidingRightPoint(UWorld* world, const FHitResult& obstacle, FVector start, UCapsuleComponent* actorCapsule, FVector& avoidingPoint)
{
    float capsuleRadius = actorCapsule->GetScaledCapsuleRadius() + 1;
    FBox box = obstacle.GetActor()->GetComponentsBoundingBox();
    float minX = box.Min.X;
    float maxX = box.Max.X;
    float minY = box.Min.Y;
    float maxY = box.Max.Y;
    if (start.X >= minX && start.Y < minY)
    {
        avoidingPoint = FVector(minX - capsuleRadius, minY - capsuleRadius, start.Z);
    }
    else if (start.X > maxX && start.Y >= minY)
    {
        avoidingPoint = FVector(maxX + capsuleRadius, minY - capsuleRadius, start.Z);
    }
    else if (start.X <= maxX && start.Y > maxY)
    {
        avoidingPoint = FVector(maxX + capsuleRadius, maxY + capsuleRadius, start.Z);
    }
    else
    {
        avoidingPoint = FVector(minX - capsuleRadius, maxY + capsuleRadius, start.Z);
    }
    return !world->OverlapBlockingTestByChannel(avoidingPoint, FQuat::Identity, ECollisionChannel::ECC_Pawn, actorCapsule->GetCollisionShape());
}


thread_local int depthRecursion = 0;

bool FindPathToLocationLeft(UWorld *world, FVector start, FVector target, FVector up, UCapsuleComponent *actorCapsule, TArray<FSplinePoint> &points, float &distance)
{
    if (depthRecursion > 4)
    {
        return false;
    }
    depthRecursion += 1;

    TArray<FHitResult> obstacles;
    bool hit = SDTUtils::SweepOverlapAgent(world, start, target, actorCapsule->GetCollisionShape(), obstacles);
    if (hit)
    {
        FVector avoidingPoint;
        if (GetAvoidingLeftPoint(world, obstacles[0], start, actorCapsule, avoidingPoint))
        {
            points.Add(FSplinePoint(points.Num() + 1, avoidingPoint));
            if (FindPathToLocationLeft(world, avoidingPoint, target, up, actorCapsule, points, distance))
            {
                distance += (avoidingPoint - start).Size();
                return true;
            }
            else
            {
                points.Pop();
            }
        }
        return false;
    }
    else
    {
        points.Add(FSplinePoint(points.Num() + 1, target));
        return true;
    }
}

bool FindPathToLocationRight(UWorld *world, FVector start, FVector target, FVector up, UCapsuleComponent *actorCapsule, TArray<FSplinePoint> &points, float& distance)
{
    if (depthRecursion > 4)
    {
        return false;
    }
    depthRecursion += 1;

    TArray<FHitResult> obstacles;
    bool hit = SDTUtils::SweepOverlapAgent(world, start, target, actorCapsule->GetCollisionShape(), obstacles);
    if (hit)
    {
        FVector avoidingPoint;
        if (GetAvoidingRightPoint(world, obstacles[0], start, actorCapsule, avoidingPoint))
        {
            points.Add(FSplinePoint(points.Num() + 1, avoidingPoint));
            if (FindPathToLocationRight(world, avoidingPoint, target, up, actorCapsule, points, distance))
            {
                distance += (avoidingPoint - start).Size();
                return true;
            }
            else
            {
                points.Pop();
            }
        }
        return false;
    }
    else
    {
        points.Add(FSplinePoint(points.Num() + 1, target));
        return true;
    }
}


bool SDTUtils::BlockingRayAgent(UWorld *world, FVector startPoint, FVector targetPoint)
{
    FHitResult hitData;
    return SDTUtils::BlockingRayAgent(world, startPoint, targetPoint, hitData);
}

bool SDTUtils::BlockingRayAgent(UWorld *world, FVector startPoint, FVector targetPoint, FHitResult &hitData)
{
    FCollisionQueryParams traceParams(FName(TEXT("Agent Trace")), true);

    return world->LineTraceSingleByChannel(hitData, startPoint, targetPoint, ECC_Pawn, traceParams);
}

bool SDTUtils::SweepOverlapAgent(UWorld *world, FVector startPoint, FVector target, const FCollisionShape &collisionShape, TArray<FHitResult> &hitData)
{
    FCollisionObjectQueryParams queryParams(ECC_TO_BITFIELD(ECC_WorldStatic) | ECC_TO_BITFIELD(COLLISION_DEATH_OBJECT));
    FCollisionQueryParams traceParams(FName(TEXT("Agent Trace")), true);

    return world->SweepMultiByObjectType(hitData, startPoint, target, FQuat::Identity, queryParams, collisionShape, traceParams);
}

bool SDTUtils::SweepBlockingAgent(UWorld *world, FVector startPoint, FVector targetPoint, const FCollisionShape &collisionShape, FHitResult &hitData)
{
    FCollisionQueryParams traceParams(FName(TEXT("Agent Trace")), true);

    return world->SweepSingleByChannel(hitData, startPoint, targetPoint, FQuat::Identity, ECollisionChannel::ECC_Pawn, collisionShape, traceParams);
}

bool SDTUtils::DetectTargetsFromAgent(UWorld *world, FVector startPoint, FVector viewDistanceDirection, float halfAngle, TArray<FOverlapResult> &overlapData)
{
    FCollisionQueryParams traceParams(FName(TEXT("Agent Trace")), true);
    float radius = viewDistanceDirection.Size();
    FCollisionShape neighborsSphere = FCollisionShape().MakeSphere(radius);
    FCollisionObjectQueryParams objectQueryParams(ECC_TO_BITFIELD(COLLISION_PLAYER) | ECC_TO_BITFIELD(COLLISION_COLLECTIBLE));

    if (world->OverlapMultiByObjectType(overlapData, startPoint, FQuat::Identity, objectQueryParams, neighborsSphere))
    {
        overlapData = overlapData.FilterByPredicate([&](auto& overlap) -> auto {
            return IsInVisionCone(startPoint, overlap.GetActor()->GetActorLocation(), viewDistanceDirection, halfAngle) &&
              !BlockingRayAgent(world, startPoint, overlap.GetActor()->GetActorLocation());
        });
        return overlapData.Num() > 0;
    }
    return false;
}

bool SDTUtils::FindPathToLocation(UWorld *world, FVector start, FVector target, FVector up, UCapsuleComponent *actorCapsule, TArray<FSplinePoint> &points)
{
    depthRecursion = 0;
    float distanceLeft = 0.0f;
    TArray<FSplinePoint> leftPathPoints;
    leftPathPoints.Add(FSplinePoint(1.0f, start));
    bool leftPathFound = FindPathToLocationLeft(world, start, target, up, actorCapsule, leftPathPoints, distanceLeft);
    depthRecursion = 0;
    float distanceRight = 0.0f;
    TArray<FSplinePoint> rightPathPoints;
    rightPathPoints.Add(FSplinePoint(1.0f, start));
    bool rightPathFound = FindPathToLocationRight(world, start, target, up, actorCapsule, rightPathPoints, distanceRight);

    if (leftPathFound && rightPathFound)
    {
        points = distanceLeft < distanceRight ? leftPathPoints : rightPathPoints;
        return true;
    }
    else if (leftPathFound)
    {
        points = leftPathPoints;
        return true;
    }
    else if (rightPathFound)
    {
        points = rightPathPoints;
        return true;
    }
    else
    {
        return false;
    }
}

bool SDTUtils::IsPlayerPoweredUp(UWorld *world)
{
    ACharacter *playerCharacter = UGameplayStatics::GetPlayerCharacter(world, 0);
    if (!playerCharacter)
        return false;

    ASoftDesignTrainingMainCharacter *castedPlayerCharacter = Cast<ASoftDesignTrainingMainCharacter>(playerCharacter);
    if (!castedPlayerCharacter)
        return false;

    return castedPlayerCharacter->IsPoweredUp();
}

bool SDTUtils::IsInVisionCone(const FVector &start, const FVector &point, const FVector &viewDistanceDirection, float halfAngle)
{
    FVector towardsPoint = point - start;
    float distance = viewDistanceDirection.Size();
    float distanceAlongCone = towardsPoint.Dot(viewDistanceDirection) / distance;
    return CosineVectors(towardsPoint, viewDistanceDirection) < halfAngle && distanceAlongCone >= 0 && distanceAlongCone <= distance;
}

float SDTUtils::CosineVectors(FVector a, FVector b)
{
    return FMath::RadiansToDegrees(FMath::Acos(a.Dot(b) / (a.Size() * b.Size())));
}
