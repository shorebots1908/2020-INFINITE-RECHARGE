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
    List hsvThresholdHue = [14.56834532374101, 48.18181818181817];
    List hsvThresholdSaturation = [181.16007194244608, 255.0];
    List hsvThresholdValue = [55.03597122302157, 197.04545454545453];

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
    Mat findBlobsInput = hsvThresholdOutput;
    Double findBlobsMinArea = 1.0;
    List findBlobsCircularity = [0.0, 1.0];
    Boolean findBlobsDarkBlobs = false;

    findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, findBlobsOutput);
}




