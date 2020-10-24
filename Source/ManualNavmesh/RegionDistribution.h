#pragma once

#include "CoreMinimal.h"
#include "RegionDistribution.generated.h"

UCLASS()
class MANUALNAVMESH_API URegionDistribution : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
public:
	UFUNCTION(BlueprintCallable, Category = "RegionDistribution")
	static void GenerateBoundedRandomPoints(int32 Count, FVector2D Min, FVector2D Max, UPARAM(ref) TArray<FVector2D>& OutPoints, int32 Seed = 0);
	UFUNCTION(BlueprintCallable, Category = "RegionDistribution")
	static void GenerateTriangulation(UPARAM(ref) TArray<FVector2D>& inPoints, UPARAM(ref) TArray<FVector2D>& OutVerts, UPARAM(ref) TArray<int32>& Triangles, UPARAM(ref) TArray<int32>& HalfEdges);

	static void DrawTriangle(UObject* WorldContextObject, int32 Index, const TArray<FVector2D>& OutVerts, const TArray<int32>& Triangles, const TArray<int32>& HalfEdges, float Inset);
};
