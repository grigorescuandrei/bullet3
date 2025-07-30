function createProject(vendor)
	hasCL = findOpenCL(vendor)
	
	if (hasCL) then
		
		project ("Bullet3OpenCL_" .. vendor)
	
		initOpenCL(vendor)
			
		kind "StaticLib"
		
        if os.is("Linux") then
            buildoptions{"-fPIC"}
        end

        if _OPTIONS["nvpro-includes"] then
            includedirs{
                _OPTIONS["vulkan_include_dir"],
                "../../../nvpro_core",
                "../../../nvpro_core/nvp",
                "../../../nvpro_core/third_party/glm",
                "../../../nvpro_core/third_party/fmt/include"
            }
		
            libdirs {
                _OPTIONS["vulkan_lib_dir"]
            }
            links {
                "nvpro_core_vk",
                "vulkan-1"
            }
        end
		
		includedirs {
			".",".."
		}
		
		files {
			"**.cpp",
			"**.h"
		}
		
	end
end

createProject("clew")
createProject("AMD")
createProject("Intel")
createProject("NVIDIA")
createProject("Apple")
