/**
  * \author Sergio Agostinho - sergio(dot)r(dot)agostinho(at)gmail(dot)com
  * \date created: 2017/07/27
  * \date last modified: 2017/07/27
  */
#include <cvl/filter/duplicate.h>
#include <cvl/io/vgm_model_reader.h>

using ht::TriMesh;

int
main (const int argc, const char** const argv)
{
  // check if input is well formed
  if (argc < 2)
  {
    std::cerr <<  "Insuficient number of arguments. "
                  "Please path to mat file"
              << std::endl;
    return -1;
  }

  TriMesh mesh;
  if (!ht::vgm_model_reader (mesh, argv[1]))
    return -1;

  std::cout << "Mesh data:\n" << mesh;

  // Filter stage
  std::shared_ptr<TriMesh>  input = std::make_shared<TriMesh> (mesh);
  ht::DuplicateVertexRemoval<TriMesh> f (1.f);
  f.setInput (input);
  mesh = f.filter ();
	return 0;
}
