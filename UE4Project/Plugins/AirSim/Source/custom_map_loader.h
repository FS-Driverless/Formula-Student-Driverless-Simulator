// This program is free software; you can redistribute it and/ormodify it under the terms of the GNU General Public License as published by the Free Software Foundation

#pragma once
#include <string>

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "custom_map_loader.generated.h"

/**
 * 
 */
UCLASS()
class AIRSIM_API Ucustom_map_loader : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	

	UFUNCTION(BlueprintCallable, Category = "custom map loader")
		static bool FileSaveString(FString SaveTextB, FString FileNameB);

	UFUNCTION(BlueprintPure, Category = "custom map loader")
		static bool FileLoadString(FString FileNameA, FString& SaveTextA);

	UFUNCTION(BlueprintPure, Category = "custom map loader")
		static TArray<FString> ProcessFile(FString data, TArray<FTransform>& blue_cones, TArray<FTransform>& yellow_cones);
};
