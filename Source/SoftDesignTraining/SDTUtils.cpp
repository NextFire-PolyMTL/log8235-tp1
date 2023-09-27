// Fill out your copyright notice in the Description page of Project Settings.

#include "SDTUtils.h"
#include "SoftDesignTraining.h"
#include "SoftDesignTrainingMainCharacter.h"
#include "DrawDebugHelpers.h"
#include "Components/CapsuleComponent.h"
#include "Engine/World.h"

#include <iostream>

bool GetAvoidingLeftPoint(UWorld *world, const FHitResult &obstacle, FVector start, UCapsuleComponent *actorCapsule, FVector &avoidingPoint)
{
    auto capsuleRadius = actorCapsule->GetScaledCapsuleRadius() + 1;
    auto box = obstacle.GetActor()->GetComponentsBoundingBox();
    auto minX = box.Min.X;
    auto maxX = box.Max.X;
    auto minY = box.Min.Y;
    auto maxY = box.Max.Y;
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

    return !world->OverlapAnyTestByObjectType(avoidingPoint, FQuat::Identity, FCollisionObjectQueryParams(ECC_TO_BITFIELD(ECC_WorldStatic) | ECC_TO_BITFIELD(COLLISION_DEATH_OBJECT)), actorCapsule->GetCollisionShape());
}

bool GetAvoidingRightPoint(UWorld *world, const FHitResult &obstacle, FVector start, UCapsuleComponent *actorCapsule, FVector &avoidingPoint)
{
    auto capsuleRadius = actorCapsule->GetScaledCapsuleRadius() + 1;
    auto box = obstacle.GetActor()->GetComponentsBoundingBox();
    auto minX = box.Min.X;
    auto maxX = box.Max.X;
    auto minY = box.Min.Y;
    auto maxY = box.Max.Y;
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
    return !world->OverlapAnyTestByObjectType(avoidingPoint, FQuat::Identity, FCollisionObjectQueryParams(ECC_TO_BITFIELD(ECC_WorldStatic) | ECC_TO_BITFIELD(COLLISION_DEATH_OBJECT)), actorCapsule->GetCollisionShape());
}

thread_local auto depthRecursion = 0;

bool FindPathToLocationLeft(UWorld *world, FVector start, FVector target, FVector up, UCapsuleComponent *actorCapsule, TArray<FVector> &points, float &distance)
{
    if (depthRecursion > 4)
    {
        return false;
    }
    depthRecursion += 1;

    TArray<FHitResult> obstacles;
    auto hit = SDTUtils::SweepOverlapAgent(world, start, target, actorCapsule->GetCollisionShape(), obstacles);
    if (hit)
    {
        FVector avoidingPoint;
        if (GetAvoidingLeftPoint(world, obstacles[0], start, actorCapsule, avoidingPoint))
        {
            points.Add(avoidingPoint);
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
        points.Add(target);
        return true;
    }
}

bool FindPathToLocationRight(UWorld *world, FVector start, FVector target, FVector up, UCapsuleComponent *actorCapsule, TArray<FVector> &points, float &distance)
{
    if (depthRecursion > 4)
    {
        return false;
    }
    depthRecursion += 1;

    TArray<FHitResult> obstacles;
    auto hit = SDTUtils::SweepOverlapAgent(world, start, target, actorCapsule->GetCollisionShape(), obstacles);
    if (hit)
    {
        FVector avoidingPoint;
        if (GetAvoidingRightPoint(world, obstacles[0], start, actorCapsule, avoidingPoint))
        {
            points.Add(avoidingPoint);
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
        points.Add(target);
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
    auto radius = viewDistanceDirection.Size();
    auto neighborsSphere = FCollisionShape().MakeSphere(radius);
    FCollisionObjectQueryParams objectQueryParams(ECC_TO_BITFIELD(COLLISION_PLAYER) | ECC_TO_BITFIELD(COLLISION_COLLECTIBLE));

    if (world->OverlapMultiByObjectType(overlapData, startPoint, FQuat::Identity, objectQueryParams, neighborsSphere))
    {
        overlapData = overlapData.FilterByPredicate([&](auto &overlap) -> auto
                                                    { return IsInVisionCone(startPoint, overlap.GetActor()->GetActorLocation(), viewDistanceDirection, halfAngle) &&
                                                             !BlockingRayAgent(world, startPoint, overlap.GetActor()->GetActorLocation()); });
        return overlapData.Num() > 0;
    }
    return false;
}

bool SDTUtils::FindPathToLocation(UWorld *world, FVector start, FVector target, FVector up, UCapsuleComponent *actorCapsule, TArray<FVector> &points)
{
    depthRecursion = 0;
    auto distanceLeft = 0.0f;
    TArray<FVector> leftPathPoints;
    leftPathPoints.Add(start);
    auto leftPathFound = FindPathToLocationLeft(world, start, target, up, actorCapsule, leftPathPoints, distanceLeft);
    depthRecursion = 0;
    auto distanceRight = 0.0f;
    TArray<FVector> rightPathPoints;
    rightPathPoints.Add(start);
    auto rightPathFound = FindPathToLocationRight(world, start, target, up, actorCapsule, rightPathPoints, distanceRight);

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
    auto playerCharacter = UGameplayStatics::GetPlayerCharacter(world, 0);
    if (!playerCharacter)
        return false;

    auto castedPlayerCharacter = Cast<ASoftDesignTrainingMainCharacter>(playerCharacter);
    if (!castedPlayerCharacter)
        return false;

    return castedPlayerCharacter->IsPoweredUp();
}

bool SDTUtils::IsInVisionCone(const FVector &start, const FVector &point, const FVector &viewDistanceDirection, float halfAngle)
{
    auto towardsPoint = point - start;
    auto distance = viewDistanceDirection.Size();
    auto distanceAlongCone = towardsPoint.Dot(viewDistanceDirection) / distance;
    return CosineVectors(towardsPoint, viewDistanceDirection) < halfAngle && distanceAlongCone >= 0 && distanceAlongCone <= distance;
}

float SDTUtils::CosineVectors(FVector a, FVector b)
{
    return FMath::RadiansToDegrees(FMath::Acos(a.Dot(b) / (a.Size() * b.Size())));
}
