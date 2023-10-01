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
    // Shift just a little bit the point outside of the corner, otherwise the agent tends to stay stuck in the corner sometimes.
    auto capsuleRadius = actorCapsule->GetScaledCapsuleRadius() + 10;
    auto box = obstacle.GetActor()->GetComponentsBoundingBox();
    auto minX = box.Min.X;
    auto maxX = box.Max.X;
    auto minY = box.Min.Y;
    auto maxY = box.Max.Y;
    // Separate the space around the box in 4 regions. Each of the region can reach one corner at its left.
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
    // Shift just a little bit the point outside of the corner, otherwise the agent tends to stay stuck in the corner sometimes.
    auto capsuleRadius = actorCapsule->GetScaledCapsuleRadius() + 10;
    auto box = obstacle.GetActor()->GetComponentsBoundingBox();
    auto minX = box.Min.X;
    auto maxX = box.Max.X;
    auto minY = box.Min.Y;
    auto maxY = box.Max.Y;
    // Separate the space around the box in 4 regions. Each of the region can reach one corner at its right.
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

    // Find if there is an obstacle between start and target.
    FHitResult hitData;
    if (!SDTUtils::SweepOverlapAgent(world, start, target, actorCapsule->GetCollisionShape(), hitData))
    {
        // There is no obstacle, we found a path.
        points.Add(target);
        distance += (target - start).Size();
        return true;
    }

    // Try to find a point on the corner of the obstacle that does not overlap another object.
    FVector avoidingPoint;
    if (!GetAvoidingLeftPoint(world, hitData, start, actorCapsule, avoidingPoint))
    {
        return false;
    }

    // Add that point and do the same logic again from that avoiding point.
    points.Add(avoidingPoint);
    distance += (avoidingPoint - start).Size();
    return FindPathToLocationLeft(world, avoidingPoint, target, up, actorCapsule, points, distance);
}

bool FindPathToLocationRight(UWorld* world, FVector start, FVector target, FVector up, UCapsuleComponent* actorCapsule, TArray<FVector>& points, float& distance)
{
    if (depthRecursion > 4)
    {
        return false;
    }
    depthRecursion += 1;

    // Find if there is an obstacle between start and target.
    FHitResult hitData;
    if (!SDTUtils::SweepOverlapAgent(world, start, target, actorCapsule->GetCollisionShape(), hitData))
    {
        // There is no obstacle, we found a path.
        points.Add(target);
        distance += (target - start).Size();
        return true;

    }

    // Try to find a point on the corner of the obstacle that does not overlap another object.
    FVector avoidingPoint;
    if (!GetAvoidingRightPoint(world, hitData, start, actorCapsule, avoidingPoint))
    {
        return false;
    }

    // Add that point and do the same logic again from that avoiding point.
    points.Add(avoidingPoint);
    distance += (target - start).Size();
    return FindPathToLocationRight(world, avoidingPoint, target, up, actorCapsule, points, distance);
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

bool SDTUtils::SweepOverlapAgent(UWorld *world, FVector startPoint, FVector target, const FCollisionShape &collisionShape, FHitResult &hitData)
{
    FCollisionObjectQueryParams queryParams(ECC_TO_BITFIELD(ECC_WorldStatic) | ECC_TO_BITFIELD(COLLISION_DEATH_OBJECT));
    FCollisionQueryParams traceParams(FName(TEXT("Agent Trace")), true);

    return world->SweepSingleByObjectType(hitData, startPoint, target, FQuat::Identity, queryParams, collisionShape, traceParams);
}

bool SDTUtils::DetectTargetsFromAgent(UWorld *world, FVector startPoint, FVector viewDistanceDirection, float halfAngle, TArray<FOverlapResult> &overlapData)
{
    FCollisionQueryParams traceParams(FName(TEXT("Agent Trace")), true);
    auto radius = viewDistanceDirection.Size();
    auto neighborsSphere = FCollisionShape().MakeSphere(radius);
    FCollisionObjectQueryParams objectQueryParams(ECC_TO_BITFIELD(COLLISION_PLAYER) | ECC_TO_BITFIELD(COLLISION_COLLECTIBLE));

    if (world->OverlapMultiByObjectType(overlapData, startPoint, FQuat::Identity, objectQueryParams, neighborsSphere))
    {
        overlapData = overlapData.FilterByPredicate([&](auto& overlap) -> auto
            { return (
                //If the collective is in the vision cone
                (IsInVisionCone(startPoint, overlap.GetActor()->GetActorLocation(), viewDistanceDirection, halfAngle) &&
                overlap.GetComponent()->GetCollisionObjectType() == COLLISION_COLLECTIBLE) ||
                //If the player is in the sensory sphere
                (IsInSensorySphere(startPoint, overlap.GetActor()->GetActorLocation(), viewDistanceDirection) &&
                overlap.GetComponent()->GetCollisionObjectType() == COLLISION_PLAYER))

                &&

                !BlockingRayAgent(world, startPoint, overlap.GetActor()->GetActorLocation());
        });
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

bool SDTUtils::IsInSensorySphere(const FVector &start, const FVector &point, const FVector &viewDistanceDirection)
{
    auto directionToPoint = point - start;
    auto distanceToPoint = directionToPoint.Size();
    auto distance = viewDistanceDirection.Size();
    return distanceToPoint <= distance;
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
