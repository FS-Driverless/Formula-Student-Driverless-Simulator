// This program is free software; you can redistribute it and/ormodify it under the terms of the GNU General Public License as published by the Free Software Foundation

#pragma once
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <vector>

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "custom_map_loader.generated.h"

/**
 * 
 */

struct Coordinate {
	float x = 0.0f;
	float y = 0.0f;
};

UCLASS()
class AIRSIM_API Ucustom_map_loader : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	


	UFUNCTION(BlueprintCallable, Category = "custom map loader")
		static bool FileLoadString(FString FileNameA, FString& SaveTextA);

	UFUNCTION(BlueprintCallable, Category = "custom map loader")
		static TArray<FString> ProcessFile(FString data, TArray<FTransform>& blue_cones, TArray<FTransform>& yellow_cones, TArray<FTransform> & big_orange_cones);

	UFUNCTION(BlueprintCallable, Category = "custom map loader")
		static FTransform GetFinishTransform(TArray<FTransform> big_orange_cones);

	UFUNCTION(BlueprintCallable, Category = "custom map loader")
		static void SetCustomMapPath(FString path);

	UFUNCTION(BlueprintCallable, Category = "custom map loader")
		static FString GetCustomMapPath();




	static float getEuclideanDistance(Coordinate coord1, Coordinate coord2);
};
