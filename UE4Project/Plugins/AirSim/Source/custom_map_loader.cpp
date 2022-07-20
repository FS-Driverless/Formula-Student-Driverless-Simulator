// This program is free software; you can redistribute it and/ormodify it under the terms of the GNU General Public License as published by the Free Software Foundation


#include "custom_map_loader.h"

bool Ucustom_map_loader::FileSaveString(FString SaveTextB, FString FileNameB)
{
	return FFileHelper::SaveStringToFile(SaveTextB, L"test.txt");
}

bool Ucustom_map_loader::FileLoadString(FString FileNameA, FString& SaveTextA)
{
	const TCHAR* file = *FileNameA;
	UE_LOG(LogTemp, Warning, TEXT("LOADING STRING"));
	return FFileHelper::LoadFileToString(SaveTextA, file);
}


struct Actor {
	FString type;
	float x;
	float y;
	float heading;
	float x_variance;
	float y_variance;
	float xy_variance;
};
TArray<FString> Ucustom_map_loader::ProcessFile(FString data, TArray<FTransform> & blue_cones, TArray<FTransform> & yellow_cones) {
	TArray<FString> lines;
	TArray<FString> values;

	FString left = "", new_left = "";
	FString right = data;

	while (right.Split("\n", &left, &right)) {
		lines.Add(left);
		
	}

	lines.Add(right);

	for (FString& line : lines) {
		FString value = "";

		Actor actor;

		line.Split(",", &actor.type, &line);
		line.Split(",", &value, &line);
		actor.x = FCString::Atof(*value) * 100;
		line.Split(",", &value, &line);
		actor.y = FCString::Atof(*value) * 100;
		line.Split(",", &value, &line);
		actor.heading = FCString::Atof(*value);
		line.Split(",", &value, &line);
		actor.x_variance = FCString::Atof(*value);
		line.Split(",", &value, &line);
		actor.y_variance = FCString::Atof(*value);
		line.Split(",", &value, &line );
		actor.xy_variance = FCString::Atof(*value);


		if (actor.type == "yellow") {
			FTransform transform{
				FRotator{},                 // Rotation
				FVector{actor.x, actor.y, 5.0f},  // Translation
				FVector{1.0f, 1.0f, 1.0f}   // Scale
			};
			yellow_cones.Add(transform);
		}

		if (actor.type == "blue") {
			FTransform transform{
				FRotator{},                 // Rotation
				FVector{actor.x, actor.y, 5.0f},  // Translation
				FVector{1.0f, 1.0f, 1.0f}   // Scale
			};
			blue_cones.Add(transform);
		}
	}

	return lines;
}