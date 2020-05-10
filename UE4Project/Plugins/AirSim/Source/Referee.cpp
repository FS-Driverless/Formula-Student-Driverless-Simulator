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
	//todo: implement a counter and update the counter based on cone hits
	return msr::airlib::CarApiBase::RefereeState(1);
}

// Called every frame
void AReferee::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

