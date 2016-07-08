#include "VtkObjReader.h"
#include <string.h>
#include <vtkTexture.h>
#include <vtkImageData.h>
#include <vtkPNGReader.h>
#include <vtkJPEGReader.h>
#include <vtkOBJReader.h>


VtkObjReader::VtkObjReader()
{
}

VtkObjReader::~VtkObjReader()
{
}

//==================================================================================================================

#define MAX_FULL_FILE_NAME_LEN		2048
#define MAX_FILE_NAME_LEN			1024
#define MAX_LINE_LEN				2048
#define MAX_MTL_NAME_LEN			128

#define OBJ_MTLLIB_STR				"mtllib"
#define OBJ_USEMTL_STR				"usemtl"
#define OBJ_DIFFUSE_TEX				"map_Kd"

#define OBJ_KEYWORD_MAX_LEN			6

void VtkObjReader::read(char **fileNames, char *folderName, int numOfFiles, std::vector<VtkPolyData*> &objects)
{
	FILE *objfp, *mtlfp;
	char fileLine[MAX_LINE_LEN], mtllibFullName[MAX_FULL_FILE_NAME_LEN], mtllibName[MAX_FILE_NAME_LEN], mtlName[MAX_MTL_NAME_LEN];
	char texFullName[MAX_FULL_FILE_NAME_LEN], texName[MAX_FILE_NAME_LEN];
	char ext[5], *str;
	bool mtllibFound, mtlFound;
	int len;
	vtkTexture *tex;

	for ( int k = 0 ; k < numOfFiles ; ++k )
	{
		// Check if we have the right file extension
		len = strlen(fileNames[k]);
		ext[0] = fileNames[k][len-3]; ext[1] = fileNames[k][len-2]; ext[2] = fileNames[k][len-1]; ext[3] = '\0';
		if ( strcmp(ext, "OBJ") && strcmp(ext, "obj") )
		{
			fprintf(stderr, "Unkown extension '%s' for file '%s'\n", ext, fileNames[k]);
			continue;
		}

		// Try to open the file
		objfp = fopen(fileNames[k], "r");
		if ( objfp == NULL )
		{
			fprintf(stderr, "Can not open file '%s'\n", fileNames[k]);
			continue;
		}

		printf("Loading file '%s' ...\n", fileNames[k]); fflush(stdout);
		mtllibFound = mtlFound = false;

		// Check whether a matlib and a material are used.
		// WARNING: only the matlib and material, that are encountered first are considered!

		// Not the most efficient way, but simple and with little memory effort
		while ( fgets(fileLine, MAX_LINE_LEN, objfp) != NULL && (!mtllibFound || !mtlFound) ) // As long as we have lines in the file
		{
			if ( strlen(fileLine) > OBJ_KEYWORD_MAX_LEN )
			{
			// Check if we have some .obj keyword
				str = strstr(fileLine, OBJ_MTLLIB_STR);
				if ( str )
				{
					this->copy(str+6, mtllibName);
					sprintf(mtllibFullName, "%s/%s", folderName, mtllibName);
					mtllibFound = true;
					continue;
				}
				str = strstr(fileLine, OBJ_USEMTL_STR);
				if ( str )
				{
					this->copy(str+6, mtlName);
					mtlFound = true;
					continue;
				}
			}
		}
		// Close the obj-file
		fclose(objfp);
		tex = NULL;

		// If a matlib and a material is used check whether the material has a texture and load it
		if ( mtllibFound && mtlFound )
		{
			mtlfp = fopen(mtllibFullName, "r");
			if ( mtlfp )
			{
				while ( fgets(fileLine, MAX_LINE_LEN, mtlfp) != NULL && tex == NULL )
				{
					// Check whether there is a texture map for the diffuse color
					str = strstr(fileLine, OBJ_DIFFUSE_TEX);
					if ( str )
					{
						this->copy(str+6, texName);
						// Construct full name of the texture file
						sprintf(texFullName, "%s/%s", folderName, texName);
						// Construct the extension of the texture file
						len = strlen(texName);
						ext[0]=texName[len-4]; ext[1]=texName[len-3]; ext[2]=texName[len-2]; ext[3]=texName[len-1]; ext[4]='\0';
						// Try to load the texure
						if ( !strcmp(ext, ".PNG") || !strcmp(ext, ".png") )
						{
							vtkPNGReader *png = vtkPNGReader::New();
							if ( png->CanReadFile(texFullName) )
							{
								png->SetFileName(texFullName);
								tex = vtkTexture::New();
								tex->SetInput(png->GetOutput());
								printf("Texture file = '%s'\n", texName);
							}
							else
								fprintf(stderr, "Can not read '%s'\n", texFullName);
							// Clean up
							png->Delete();
						}
						// Try to load the texure
						else if ( !strcmp(ext, ".JPG") || !strcmp(ext, ".jpg") || !strcmp(ext, "jpeg") || !strcmp(ext, "JPEG") )
						{
							vtkJPEGReader *jpeg = vtkJPEGReader::New();
							if ( jpeg->CanReadFile(texFullName) )
							{
								jpeg->SetFileName(texFullName);
								tex = vtkTexture::New();
								tex->SetInput(jpeg->GetOutput());
								printf("Texture file = '%s'\n", texName);
							}
							else
								fprintf(stderr, "Can not read '%s'\n", texFullName);
							// Clean up
							jpeg->Delete();
						}
						else
							fprintf(stderr, "Not supported image format: '%s'\n", ext);
					}
				}
			}
			else
				fprintf(stderr, "Can not open material library '%s'\n", mtllibFullName);
		}

		// Load the geometry of the obj-file
		vtkOBJReader *objReader = vtkOBJReader::New();
		objReader->SetFileName(fileNames[k]);
		objReader->Update();
		VtkPolyData *polyData = new VtkPolyData(objReader->GetOutput());

		if ( tex )
			polyData->getActor()->SetTexture(tex);

		objects.push_back(polyData);

		// Clean up
		objReader->Delete();
	}
}

//===================================================================================================================================

void VtkObjReader::copy(const char *src, char *dst)
{
	int i, j;
	for ( i = 0, j = 0 ; src[i] != '\0' ; ++i )
		if ( src[i] > 32 )
			dst[j++] = src[i];

	dst[j] = '\0';
}

//===================================================================================================================================
