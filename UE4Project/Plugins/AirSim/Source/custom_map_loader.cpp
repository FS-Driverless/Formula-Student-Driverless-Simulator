// This program is free software; you can redistribute it and/ormodify it under the terms of the GNU General Public License as published by the Free Software Foundation


#include "custom_map_loader.h"

static FString custom_map_path;


bool Ucustom_map_loader::FileLoadString(FString FileNameA, FString& SaveTextA)
{
	const TCHAR* file = *FileNameA;
	UE_LOG(LogTemp, Warning, TEXT("Loading file %s"), *FileNameA);
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
	// Get random seed for rotation of cones
	srand((unsigned)time(NULL));

	// Read the lines
	TArray<FString> lines;
	TArray<FString> values;

	FString left = "", new_left = "";
	FString right = data;

	while (right.Split("\n", &left, &right)) {
		lines.Add(left);
		
	}

	lines.Add(right);

	// Process each line
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
				FRotator{0.0f, float(rand() % 360), 0.0f},      // Rotation
				FVector{actor.x, -actor.y, 5.0f},				// Translation
				FVector{1.0f, 1.0f, 1.0f}						// Scale
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
		angle = (std::atan2f(big_orange_cones[0].GetLocation().Y - big_orange_cones[1].GetLocation().Y, big_orange_cones[0].GetLocation().X - big_orange_cones[1].GetLocation().X) - PI / 2) * 180 / PI;
	}
	else if (big_orange_cones.Num() == 4) {
		// Get middle points of all sides of rectangle

		int max_index = 0;
		float max_value = 0.0f;
		std::vector<Coordinate> midpoints;

		// Get first mid points from first orange cone
		Coordinate coord_base{ big_orange_cones[0].GetLocation().X , big_orange_cones[0].GetLocation().Y };
		for (int j = 1; j < big_orange_cones.Num(); j++) {
			Coordinate coord;
			coord.x = (big_orange_cones[j].GetLocation().X + big_orange_cones[0].GetLocation().X) / 2;
			coord.y = (big_orange_cones[j].GetLocation().Y + big_orange_cones[0].GetLocation().Y) / 2;

			float distance = getEuclideanDistance(coord_base, coord);

			if (distance > max_value) {
				max_value = distance;
				max_index = j - 1;
			}

			midpoints.push_back(coord);
		}

		// Remove point with biggest distance, this should be the diagonal
		midpoints.erase(midpoints.begin() + max_index);

		// Explore other midpoints from furthest point (opposite side of rectangle with respect to big_orange_cones[0])
		int current_index = max_index + 1;
		max_value = 0.0f;
		int local_index = 0;
		coord_base.x = big_orange_cones[current_index].GetLocation().X;
		coord_base.y = big_orange_cones[current_index].GetLocation().Y;
		for (int j = 1; j < big_orange_cones.Num(); j++) {
			if (j == current_index) {
				continue;
			}

			Coordinate coord;
			coord.x = (big_orange_cones[j].GetLocation().X + big_orange_cones[current_index].GetLocation().X) / 2;
			coord.y = (big_orange_cones[j].GetLocation().Y + big_orange_cones[current_index].GetLocation().Y) / 2;

			midpoints.push_back(coord);
		}


		// get longest distance between midpoints
		std::pair<Coordinate, Coordinate> start_line;
		max_value = 0.0f;
		for (int i = 0; i < midpoints.size(); i++) {
			for (int j = i + 1; j < midpoints.size(); j++) {
				float distance = getEuclideanDistance(midpoints[i], midpoints[j]);

				if (distance > max_value) {
					max_value = distance;
					start_line = std::make_pair(midpoints[i], midpoints[j]);
				}
			}
		}

		angle = (std::atan2f(start_line.second.y - start_line.first.y, start_line.second.x - start_line.first.x) - PI / 2) * 180 / PI;
	}
	else {
		UE_LOG(LogTemp, Warning, TEXT("Only start finish lines with 2 or 4 big orange cones are supported. Angle of the start finish line will probably be incorrect."));
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


float Ucustom_map_loader::getEuclideanDistance(Coordinate coord1, Coordinate coord2) {
	return std::sqrt((coord2.x - coord1.x) * (coord2.x - coord1.x) + (coord2.y - coord1.y) * (coord2.y - coord1.y));
}


void Ucustom_map_loader::SetCustomMapPath(FString path) {
	custom_map_path = path;
}


FString Ucustom_map_loader::GetCustomMapPath() {
	return custom_map_path;
}
