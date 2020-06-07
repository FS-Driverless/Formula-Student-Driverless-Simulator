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

float AReferee::AppendCone(FTransform transform) {
	float x = transform.GetTranslation().X;
	std::cout << "----------------------------" << std::endl;
	std::cout << x << std::endl;
	std::cout << "----------------------------" << std::endl;

	return x;
}