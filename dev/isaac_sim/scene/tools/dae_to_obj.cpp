#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: dae_to_obj INPUT.dae OUTPUT.obj\n";
    return 2;
  }

  const std::filesystem::path input = std::filesystem::absolute(argv[1]);
  const std::filesystem::path output = std::filesystem::absolute(argv[2]);
  std::error_code error;
  std::filesystem::create_directories(output.parent_path(), error);
  if (error) {
    std::cerr << "cannot create output directory: " << error.message() << "\n";
    return 3;
  }

  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(
      input.string(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                          aiProcess_GenSmoothNormals | aiProcess_ImproveCacheLocality |
                          aiProcess_SortByPType | aiProcess_ValidateDataStructure);
  if (scene == nullptr) {
    std::cerr << "Assimp import failed for " << input << ": " << importer.GetErrorString() << "\n";
    return 4;
  }

  Assimp::Exporter exporter;
  const aiReturn result = exporter.Export(scene, "obj", output.string());
  if (result != aiReturn_SUCCESS) {
    std::cerr << "Assimp export failed for " << output << ": " << exporter.GetErrorString() << "\n";
    return 5;
  }
  if (!std::filesystem::is_regular_file(output) || std::filesystem::file_size(output) == 0) {
    std::cerr << "Assimp reported success but wrote no OBJ: " << output << "\n";
    return 6;
  }

  std::cout << output << "\n";
  return 0;
}
