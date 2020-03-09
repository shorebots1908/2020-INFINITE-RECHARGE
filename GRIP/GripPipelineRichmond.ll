//
// Inputs
//
Inputs
{
	Mat source0;
}

//
// Variables
//
Outputs
{
	Mat blurOutput;
	Mat hsvThresholdOutput;
	Mat maskOutput;
	BlobsReport findBlobsOutput;
}

//
// Steps
//

Step Blur0
{
    Mat blurInput = source0;
    BlurType blurType = BOX;
    Double blurRadius = 2.7027027027027017;

    blur(blurInput, blurType, blurRadius, blurOutput);
}

Step HSV_Threshold0
{
    Mat hsvThresholdInput = blurOutput;
    List hsvThresholdHue = [1.618705035971223, 52.72727272727272];
    List hsvThresholdSaturation = [181.16007194244608, 255.0];
    List hsvThresholdValue = [45.863309352517966, 244.26767676767673];

    hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);
}

Step Mask0
{
    Mat maskInput = blurOutput;
    Mat maskMask = hsvThresholdOutput;

    mask(maskInput, maskMask, maskOutput);
}

Step Find_Blobs0
{
    Mat findBlobsInput = maskOutput;
    Double findBlobsMinArea = 4.0;
    List findBlobsCircularity = [0.17985611510791366, 1.0];
    Boolean findBlobsDarkBlobs = false;

    findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, findBlobsOutput);
}




