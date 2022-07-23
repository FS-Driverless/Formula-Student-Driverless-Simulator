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
TArray<FString> Ucustom_map_loader::ProcessFile(FString data, TArray<FTransform> & blue_cones, TArray<FTransform> & yellow_cones, TArray<FTransform> & big_orange_cones) {
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

		FTransform transform{
				FRotator{0.0f, 0.0f, 0.0f},       // Rotation
				FVector{actor.x, actor.y, 5.0f},  // Translation
				FVector{1.0f, 1.0f, 1.0f}         // Scale
		};

		
		if (actor.type == "yellow") {
			yellow_cones.Add(transform);
		}

		if (actor.type == "blue") {
			blue_cones.Add(transform);
		}

		if (actor.type == "big_orange") {
			big_orange_cones.Add(transform);
		}
	}

	return lines;
}


float Ucustom_map_loader::getDist(FTransform& a1, FTransform& a2) {
	// Euclidean distance
	float dist = std::sqrt((a1.GetLocation().X - a2.GetLocation().X) * (a1.GetLocation().X - a2.GetLocation().X) + (a1.GetLocation().Y - a2.GetLocation().Y) * (a1.GetLocation().Y - a2.GetLocation().Y));

	return dist;
}

FTransform Ucustom_map_loader::GetFinishTransform(TArray<FTransform> big_orange_cones) {

	FVector location{0.0f, 0.0f, 0.0f};

	for (FTransform & cone: big_orange_cones) {
		location.X += cone.GetLocation().X;
		location.Y += cone.GetLocation().Y;
	}

	location.X /= big_orange_cones.Num();
	location.Y /= big_orange_cones.Num();

	float angle = 0.0f;
	if (big_orange_cones.Num() == 2) {
		angle = (std::atan2f(big_orange_cones[0].GetLocation().Y - big_orange_cones[1].GetLocation().Y, big_orange_cones[0].GetLocation().X - big_orange_cones[1].GetLocation().X) - PI/2) * 180 / PI;
	}

	UE_LOG(LogTemp, Warning, TEXT("x1: %f, x2: %f, y1: %f, y2: %f"), big_orange_cones[0].GetLocation().X, big_orange_cones[1].GetLocation().X, big_orange_cones[0].GetLocation().Y, big_orange_cones[1].GetLocation().Y);

	UE_LOG(LogTemp, Warning, TEXT("Angle: %f"), angle);

	FTransform transform{
		FRotator{0.0f, angle, 0.0f},
		location,
		FVector{1.0f, 1.0f, 1.0f}
	};

	return transform;
}