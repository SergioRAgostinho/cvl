/**
  * \author Sergio Agostinho - sergio.r.agostinho@gmail.com
  * \date created: 2017/07/25
  * \date last modified: 2017/07/26
  */
#include <mat.h> //MATLAB provided header
#include <iostream>

int
main (const int argc, const char** const argv)
{
  if (argc < 2)
  {
    std::cerr <<  "Insufficient arguments. Usage: "
                  "mat_file_io <path to mat file>"
              << std::endl;
    return -1;
  }

  // Open file
  std::clog << "Opening file " << argv[1] << std::endl;
  MATFile* const mat = matOpen(argv[1], "r");
  if (!mat)
  {
    std::cerr << "Failed to open mat file." << std::endl;
    return -1;
  }

  //Get directory and print info
  int ndir;
  const char** const dir  = (const char **) matGetDir (mat, &ndir);
  std::cout << "Directory of " << argv[1] << ':' << std::endl;
  for (int i = 0; i < ndir; ++i)
  {
    mxArray *aPtr = matGetVariable (mat, dir[i]);
    std::cout << dir[i] << " - " <<  mxGetClassName (aPtr)
              << std::endl;

    if (mxGetClassID(aPtr) == mxSTRUCT_CLASS)
    {
      const int n = mxGetNumberOfFields (aPtr);

      // print fields
      for (int j = 0; j < n; ++j)
      {
        mxArray* const field = mxGetFieldByNumber (aPtr, 0, j);
        const mwSize* const dims = mxGetDimensions (field);
        const mwSize size = mxGetNumberOfDimensions (field);
        std::cout << ' ' << mxGetFieldNameByNumber (aPtr, j)
                  << " - " << mxGetClassName (field)
                  << " (" << dims[0];
        for (unsigned int k = 1; k < size; ++k)
          std::cout << 'x' << dims[k];
        std::cout << ')' << std::endl;

        const double* const data = (const double*) mxGetData (field);
        for (unsigned int k = 0; k < dims[1]; ++k)
          std::cout << ' ' << data[3*k]
                    << ' ' << data[3*k + 1]
                    << ' ' << data[3*k + 2]
                    << std::endl;
      }
    }

    mxDestroyArray(aPtr);
  }
  mxFree(dir);

  // Close file and cleanup
  if (matClose (mat))
  {
    std::cerr << "Failed to close the mat file" << std::endl;
    return -1;
  }
	return 0;
}
