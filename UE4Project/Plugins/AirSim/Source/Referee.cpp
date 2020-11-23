// This program is free software; you can redistribute it and/ormodify it under the terms of the GNU General Public License as published by the Free Software Foundation

#include "Referee.h"


// Sets default values
AReferee::AReferee()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AReferee::BeginPlay()
{
	Super::BeginPlay();
	
}

msr::airlib::CarApiBase::RefereeState AReferee::getState() {
	return this->state;
}

// Called every frame
void AReferee::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

int32 AReferee::ConeHit(FString coneName)
{
	return (int32) (++state.doo_counter);
}

int32 AReferee::LapCompleted(float lapTime)
{
	state.laps.push_back(lapTime);
	return state.laps.size();
}

void AReferee::AppendYellowCone(FTransform transform) {
	msr::airlib::CarApiBase::Cone cone;
	cone.location.x = transform.GetTranslation().X;
	cone.location.y = transform.GetTranslation().Y;
	cone.color = msr::airlib::CarApiBase::ConeColor::Yellow;
	state.cones.push_back(cone);
}

void AReferee::AppendBlueCone(FTransform transform) {
	msr::airlib::CarApiBase::Cone cone;
	cone.location.x = transform.GetTranslation().X;
	cone.location.y = transform.GetTranslation().Y;
	cone.color = msr::airlib::CarApiBase::ConeColor::Blue;
	state.cones.push_back(cone);
}

void AReferee::AppendBigOrangeCone(FTransform transform){
	msr::airlib::CarApiBase::Cone cone;
	cone.location.x = transform.GetTranslation().X;
	cone.location.y = transform.GetTranslation().Y;
	cone.color = msr::airlib::CarApiBase::ConeColor::OrangeLarge;
	state.cones.push_back(cone);
}

void AReferee::LoadStartPos(FVector pos) {
	msr::airlib::CarApiBase::Point2D point;
	point.x = pos.X;
	point.y = pos.Y;
	state.car_start_location = point;
}