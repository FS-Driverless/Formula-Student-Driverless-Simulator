// This program is free software; you can redistribute it and/ormodify it under the terms of the GNU General Public License as published by the Free Software Foundation


#include "custom_map_loader.h"

bool Ucustom_map_loader::FileSaveString(FString SaveTextB, FString FileNameB)
{
	return FFileHelper::SaveStringToFile(SaveTextB, L"test.txt");
}

bool Ucustom_map_loader::FileLoadString(FString FileNameA, FString& SaveTextA)
{
	const TCHAR* file = *FileNameA;
	return FFileHelper::LoadFileToString(SaveTextA, file);
}

TArray<FString> Ucustom_map_loader::ProcessFile(FString data, TArray<FTransform> & blue_cones, TArray<FTransform> & yellow_cones) {
	TArray<FString> lines;
	
	FString left = "";
	FString right = data;

	while (right.Split("\n", &left, &right)) {
		lines.Add(left);
		std::string line = TCHAR_TO_UTF8(&left);

		FString fcolor(line.c_str());
		UE_LOG(LogTemp, Warning, TEXT("The Actor's name is %s"), *fcolor);

		//char color[100];
		//float x = 0, y = 0, z = 0;
		//std::sscanf(line.c_str(), "%s %f %f %f", color, &x, &y, &z);
		

		//UE_LOG(LogTemp, Warning, TEXT("%s"), *color);
	}

	lines.Add(right);

	return lines;
}