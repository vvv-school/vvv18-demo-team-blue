# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# closest-blob.thrift

/**
* closest-blob_IDL
*
* IDL Interface to \ref Closest Blob Module.
*/
service closestBlob_IDL
{
    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();

     /*

     */
     bool setGausianSize(1:i32 size);



    /*
    */
    bool setCoefThreshold(1:double t_coef);


    /*
    */
    bool setCannyParam(1:i32 t_canny);

    bool setErodeParam(1:i32 t_canny);

    bool setDilateParam(1:i32 t_canny);

    /*
    */
    i32 getGausianSize();


    /*
    */
    double getCoefThreshold();

    /*
    */
    i32 getCannyParam();

    i32 getErodeParam();

    i32 getDilateParam();

    /*

     */
    list<double> get3DPoint(1:i32 t_canny);



}
