// Copyright (c) RAMMP. All rights reserved.

using UnrealBuildTool;

public class RammsAccess : ModuleRules
{
	public RammsAccess(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"Json",
			"Sockets",
			"Networking",
			"RammsCore", // drive / arm / gripper controllers the intents map onto
		});
	}
}
