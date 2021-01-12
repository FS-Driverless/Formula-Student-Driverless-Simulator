// This program is free software; you can redistribute it and/ormodify it under the terms of the GNU General Public License as published by the Free Software Foundation

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "vehicles/car/api/CarApiBase.hpp"
#include "Referee.generated.h"

UCLASS()
class AIRSIM_API AReferee : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AReferee();

	msr::airlib::CarApiBase::RefereeState getState();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	msr::airlib::CarApiBase::RefereeState state;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	UFUNCTION(BlueprintCallable, Category="Referee")
	int32 ConeHit(FString coneName);

	UFUNCTION(BlueprintCallable, Category="Referee")
	int32 LapCompleted(float lapTime);

	UFUNCTION(BlueprintCallable, Category="Referee")
	void AppendYellowCone(FTransform cone);

	UFUNCTION(BlueprintCallable, Category="Referee")
	void AppendBlueCone(FTransform cone);

	UFUNCTION(BlueprintCallable, Category="Referee")
	void AppendBigOrangeCone(FTransform cone);

	UFUNCTION(BlueprintCallable, Category="Referee")
	void LoadStartPos(FVector pos);
};
