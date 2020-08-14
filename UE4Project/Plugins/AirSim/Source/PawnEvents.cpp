#include "PawnEvents.h"

PawnEvents::CollisionSignal& PawnEvents::getCollisionSignal()
{
    return collision_signal_;
}

PawnEvents::PawnTickSignal& PawnEvents::getPawnTickSignal()
{
    return pawn_tick_signal_;

}

PawnEvents::PawnTickSignal& PawnEvents::getPawnSubtickSignal()
{
    return pawn_subtick_signal_;
}