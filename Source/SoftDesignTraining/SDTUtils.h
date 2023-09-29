// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/SplineComponent.h"

class UCapsuleComponent;

constexpr ECollisionChannel COLLISION_DEATH_OBJECT = ECollisionChannel::ECC_GameTraceChannel3;
constexpr ECollisionChannel COLLISION_PLAYER = ECollisionChannel::ECC_GameTraceChannel4;
constexpr ECollisionChannel COLLISION_COLLECTIBLE = ECollisionChannel::ECC_GameTraceChannel5;

class SOFTDESIGNTRAINING_API SDTUtils
{
public:
    /// Trace a ray between the sourcePoint and targetPoint to find the first blocking hit.
    /// The blocking objects are the ones defined by the channel ECC_Pawn.
    /// \param uWorld A valid world pointer to launch the ray.
    /// \param startPoint The point where to start the ray.
    /// \param targetPoint The point where to ray stops.
    /// \return True if a collision occured, false otherwise.
    static bool BlockingRayAgent(UWorld *world, FVector startPoint, FVector targetPoint);

    /// Trace a ray between the sourcePoint and targetPoint and provide the first blocking hit.
    /// The blocking objects are the ones defined by the channel ECC_Pawn.
    /// \param uWorld A valid world pointer to launch the ray.
    /// \param startPoint The point where to start the ray.
    /// \param targetPoint The point where to ray stops.
    /// \param hitData [out] The hit information of the ray.
    /// \return True if a collision occured, false otherwise.
    static bool BlockingRayAgent(UWorld *world, FVector startPoint, FVector targetPoint, FHitResult &hitData);

    /// Sweep the shape from a start point to a target point to find all the overlaped objects and a blocking object.
    /// The overlaped objects and blocking objects are the ones defined by the channel ECC_Pawn.
    /// \param world A valid world pointer to launch the sweep.
    /// \param startPoint The point where to start the sweep.
    /// \param targetPoint The point where to stop the sweep.
    /// \param collisionShape The shape sweep inside the world.
    /// \param hitData [out] The overlap hits followed by a blocking hit, if any.
    /// \return True if an overlap or collision occured, false otherwise.
    static bool SweepOverlapAgent(UWorld *world, FVector startPoint, FVector target, const FCollisionShape &collisionShape, TArray<FHitResult> &hitData);

    /// Sweep a shape from a start point to a target point and provide the first blocking hit.
    /// The blocking objects are the ones defined by the channel ECC_Pawn.
    /// \param world A valid world pointer to launch the sweep.
    /// \param startPoint The point where to start the sweep.
    /// \param targetPoint The point where to stop the sweep.
    /// \param collisionShape The shape sweep inside the world.
    /// \param hitData [out] The hit information of the sweep.
    /// \return True if a collision occured, false otherwise.
    static bool SweepBlockingAgent(UWorld *world, FVector startPoint, FVector targetPoint, const FCollisionShape &collisionShape, FHitResult &hitData);

    /// Find all the visible targets from an agent perspective. The visible targets are detected in a cone with the specified angle.
    /// The vision is blocked by the blocking objects defined by the channel ECC_Pawn.
    /// \param world A valid world pointer to launch the sweep.
    /// \param startPoint The starting point of the vision cone.
    /// \param viewDistanceDirection The direction and length of the cone.
    /// \param angle The angle in degrees of half of the cone.
    /// \param overlapData [out] The objects in the cone and visible from the startPoint.
    static bool DetectTargetsFromAgent(UWorld *world, FVector startPoint, FVector viewDistanceDirection, float halfAngle, TArray<FOverlapResult> &overlapData);

    /// Find a set of points that allows moving from start to target without colliding with an object.
    /// The blocking objects considered are the ones defined by the channel ECC_Pawn.
    /// \param world A valid world pointer to find the destination points.
    /// \param start The location where to start the path.
    /// \param target The location where to stop the path.
    /// \param actorCapsule The capsule of the agent used for detecting collisions.
    /// \param points [out] The points that traces a path between start and target in world coordinates.
    /// \return True if a path is found, false otherwise.
    static bool FindPathToLocation(UWorld *world, FVector start, FVector target, FVector up, UCapsuleComponent *actorCapsule, TArray<FVector> &points);

    /// Determine if the player has taken a power up.
    /// \param world A valid world pointed used to get the character.
    /// \return True if the player is powered up, false otherwise.
    static bool IsPlayerPoweredUp(UWorld *world);

    /// Determine if a point is inside a vision cone.
    /// \param start The starting point of the cone.
    /// \param point The point to test.
    /// \param viewDistanceDirection The direction and length of the cone.
    /// \param angle The angle in degrees of half of the cone.
    /// \return True if the point is in the cone, false otherwise.
    static bool IsInVisionCone(const FVector &start, const FVector &point, const FVector &viewDistanceDirection, float halfAngle);

    /// Determine if a point is inside a sensory sphere.
    /// \param start The center of the sphere.
    /// \param point The point to test.
    /// \param viewDistanceDirection The radius of the sphere.
    static bool IsInSensorySphere(const FVector& start, const FVector& point, const FVector& viewDistanceDirection);

    /// Calculate the angle between two vectors. The angle calculated is always positive.
    /// \param a The first vector where the angle start.
    /// \param b The second vector where the angle finishes.
    /// \param The angle in degrees from 0 to 180 inclusive.
    static float CosineVectors(FVector a, FVector b);

private:
    SDTUtils() = delete;
};
