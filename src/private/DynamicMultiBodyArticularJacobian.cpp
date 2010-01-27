/* @doc Computation of the dynamic aspect for a robot.
   This class will load the description of a robot from a VRML file
   following the OpenHRP syntax. Using ForwardVelocity it is then
   possible specifying the angular velocity and the angular value
   to get the absolute position, and absolute velocity of each
   body separetly. Heavy rewriting from the original source
   of Adrien and Jean-Remy.

   This implantation is an updated based on a mixture between
   the code provided by Jean-Remy and Adrien.

   Copyright (c) 2005-2006,
   @author Olivier Stasse, Ramzi Sellouati, Jean-Remy Chardonnet, Adrien Escande, Abderrahmane Kheddar
   Copyright (c) 2007-2009
   @author Olivier Stasse, Oussama Kannoun, Fumio Kanehiro.
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please refers to file License.txt for details on the license.

*/

/*! System includes */
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>

#include "Debug.h"

/*! Local library includes. */
#include "MatrixAbstractLayer/MatrixAbstractLayer.h"
#include "dynamicsJRLJapan/DynamicBody.h"
#include "DynMultiBodyPrivate.h"
#include "robotDynamics/jrlBody.h"

#include "fileReader.h"

using namespace dynamicsJRLJapan;

bool DynMultiBodyPrivate::getJacobian ( const CjrlJoint& inStartJoint,
                                        const CjrlJoint& inEndJoint,
                                        const vector3d& inFrameLocalPosition,
                                        matrixNxP& outjacobian,
                                        unsigned int outOffset,
                                        bool includeFreeFlyer )
{
    unsigned int valNumberDof = ( includeFreeFlyer==true ) ?numberDof() :numberDof()-6;
    unsigned int lengthJacobian = MAL_MATRIX_NB_COLS ( outjacobian );
    if ( ( MAL_MATRIX_NB_ROWS ( outjacobian ) != 6 ) ||
            ( lengthJacobian < valNumberDof + outOffset ) )
        return false;

    //This function should not erase the contents of argument outjacobian
    //MAL_MATRIX_FILL ( outjacobian,0.0 );

    unsigned int i,j;

    //determine participating joints
    std::vector<CjrlJoint *> robotRoot2StartJoint, robotRoot2EndJoint;
    JointPrivate* StartJoint = ( JointPrivate* ) ( &inStartJoint );
    JointPrivate* EndJoint = ( JointPrivate* ) ( &inEndJoint );
    robotRoot2StartJoint = StartJoint->jointsFromRootToThis();
    robotRoot2EndJoint = EndJoint->jointsFromRootToThis();

    unsigned int offset = 1;
    unsigned int minChain = ( robotRoot2StartJoint.size() <robotRoot2EndJoint.size() )
                            ?robotRoot2StartJoint.size() :robotRoot2EndJoint.size();

    for ( i=1; i< minChain; i++ )
    {
        if ( ( robotRoot2StartJoint[i]==robotRoot2EndJoint[i] ) )
            offset++;
    }

    unsigned int rank;
    JointPrivate* aJoint;
    DynamicBodyPrivate* aBody = EndJoint->linkedDBody();
    tempP = aBody->p + MAL_S3x3_RET_A_by_B ( aBody->R, inFrameLocalPosition );

    for ( i=offset;i<robotRoot2EndJoint.size();i++ )
    {
        aJoint= ( JointPrivate * ) robotRoot2EndJoint[i];
        aBody=  aJoint->linkedDBody();
        if ( includeFreeFlyer )
            rank = aJoint->rankInConfiguration()+outOffset;
        else
            rank = aJoint->rankInConfiguration()-6+outOffset;

        tempDP = tempP - aBody->p;

        switch ( aJoint->type() )
        {
            case JointPrivate::REVOLUTE_JOINT:
                MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
                for ( j=0;j<3;j++ )
                {
                    outjacobian ( j,rank ) = tempLV[j];
                    outjacobian ( j+3,rank ) = aBody->w_a[j];
                }
                break;
            case JointPrivate::PRISMATIC_JOINT:
                for ( j=0;j<3;j++ )
                {
                    outjacobian ( j,rank ) = aBody->w_a[j];
                }
                break;
            case JointPrivate::FREE_JOINT:
                for ( j=0;j<3;j++ )
                {
                    outjacobian ( j,rank+j ) = 1.0;
                    outjacobian ( j+3,rank+j+3 ) = 1.0;
                }

                outjacobian ( 0,rank+3 ) = 0.0;
                outjacobian ( 0,rank+4 ) = tempDP[2];
                outjacobian ( 0,rank+5 ) = -tempDP[1];
                outjacobian ( 1,rank+3 ) = -tempDP[2];
                outjacobian ( 1,rank+4 ) = 0.0;
                outjacobian ( 1,rank+5 ) = tempDP[0];

                outjacobian ( 2,rank+3 ) = tempDP[1];
                outjacobian ( 2,rank+4 ) = -tempDP[0];
                outjacobian ( 2,rank+5 ) = 0.0;
                break;
        }
    }

    for ( i=offset;i<robotRoot2StartJoint.size();i++ )
    {
        aJoint = ( JointPrivate * ) robotRoot2StartJoint[i];
        aBody=  aJoint->linkedDBody();
        if ( includeFreeFlyer )
            rank = aJoint->rankInConfiguration()+outOffset;
        else
            rank = aJoint->rankInConfiguration()-6+outOffset;

        tempDP = tempP - aBody->p;

        switch ( aJoint->type() )
        {
            case JointPrivate::REVOLUTE_JOINT:
                MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
                for ( j=0;j<3;j++ )
                {
                    outjacobian ( j,rank ) = -tempLV[j];
                    outjacobian ( j+3,rank ) = -aBody->w_a[j];

                }
                break;
            case JointPrivate::PRISMATIC_JOINT:
                for ( j=0;j<3;j++ )
                {
                    outjacobian ( j,rank ) = -aBody->w_a[j];
                }
                break;
            case JointPrivate::FREE_JOINT:
                for ( j=0;j<3;j++ )
                {
                    outjacobian ( j,rank+j ) = -1.0;
                    outjacobian ( j+3,rank+j+3 ) = -1.0;
                }
                outjacobian ( 0,rank+3 ) = 0.0;
                outjacobian ( 0,rank+4 ) = -tempDP[2];
                outjacobian ( 0,rank+5 ) = tempDP[1];
                outjacobian ( 1,rank+3 ) = tempDP[2];
                outjacobian ( 1,rank+4 ) = 0.0;
                outjacobian ( 1,rank+5 ) = -tempDP[0];
                outjacobian ( 2,rank+3 ) = -tempDP[1];
                outjacobian ( 2,rank+4 ) = tempDP[0];
                outjacobian ( 2,rank+5 ) = 0.0;
                break;
        }
    }

    if ( includeFreeFlyer )
    {
        tempDP = tempP - StartJoint->linkedDBody()->p;

        unsigned int ii,jj;
        for (ii=0;ii<6;ii++)
        {
            for (jj=outOffset;jj<outOffset+6;jj++)
                outjacobian(ii,jj) = 0.0;
            outjacobian(ii,outOffset+ii) = 1.0;
        }

        unsigned k = outOffset+3;
        outjacobian ( 1,k ) = -tempDP[2]; outjacobian ( 2,k ) = tempDP[1];k++;
        outjacobian ( 0,k ) = tempDP[2]; outjacobian ( 2,k ) = -tempDP[0];k++;
        outjacobian ( 0,k ) =  -tempDP[1]; outjacobian ( 1,k ) =  tempDP[0];;

    }

    return true;
}

bool DynMultiBodyPrivate::getPositionJacobian ( const CjrlJoint& inStartJoint,
        const CjrlJoint& inEndJoint,
        const vector3d& inFrameLocalPosition,
        matrixNxP& outjacobian,
        unsigned int outOffset,
        bool includeFreeFlyer )
{
    unsigned int valNumberDof = ( includeFreeFlyer==true ) ?numberDof() :numberDof()-6;
    unsigned int lengthJacobian = MAL_MATRIX_NB_COLS ( outjacobian );

    if ( ( MAL_MATRIX_NB_ROWS ( outjacobian ) != 3 ) || ( lengthJacobian < valNumberDof + outOffset ) )
        return false;

    unsigned int i,j;
    ////This function should not erase the contents of argument outJacobian
    //MAL_MATRIX_FILL ( outjacobian,0.0 );

    //determine participating joints
    std::vector<CjrlJoint *> robotRoot2StartJoint, robotRoot2EndJoint;
    JointPrivate* StartJoint = ( JointPrivate* ) ( &inStartJoint );
    JointPrivate* EndJoint = ( JointPrivate* ) ( &inEndJoint );
    robotRoot2StartJoint = StartJoint->jointsFromRootToThis();
    robotRoot2EndJoint = EndJoint->jointsFromRootToThis();

    unsigned int offset = 1;
    unsigned int minChain = ( robotRoot2StartJoint.size() <robotRoot2EndJoint.size() )
                            ?robotRoot2StartJoint.size() :robotRoot2EndJoint.size();

    for ( i=1; i< minChain; i++ )
    {
        if ( ( robotRoot2StartJoint[i]==robotRoot2EndJoint[i] ) )
            offset++;
    }

    unsigned int rank;
    JointPrivate* aJoint;
    DynamicBodyPrivate* aBody = EndJoint->linkedDBody();
    tempP = aBody->p + MAL_S3x3_RET_A_by_B ( aBody->R, inFrameLocalPosition );

    for ( i=offset;i<robotRoot2EndJoint.size();i++ )
    {
        aJoint= ( JointPrivate * ) robotRoot2EndJoint[i];
        aBody=  aJoint->linkedDBody();
        if ( includeFreeFlyer )
            rank = aJoint->rankInConfiguration()+outOffset;
        else
            rank = aJoint->rankInConfiguration()-6+outOffset;

        tempDP = tempP - aBody->p;

        switch ( aJoint->type() )
        {
            case JointPrivate::REVOLUTE_JOINT:
                MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
                for ( j=0;j<3;j++ )
                    outjacobian ( j,rank ) =  tempLV[j];
                break;
            case JointPrivate::PRISMATIC_JOINT:
                for ( j=0;j<3;j++ )
                    outjacobian ( j,rank ) = aBody->w_a[j];
                break;
            case JointPrivate::FREE_JOINT:
                for ( j=0;j<3;j++ )
                    outjacobian ( j,rank+j ) = 1.0;
                outjacobian ( 1,rank+3 ) =  -tempDP[2];
                outjacobian ( 2,rank+3 ) =  tempDP[1];
                outjacobian ( 0,rank+4 ) =  tempDP[2];
                outjacobian ( 2,rank+4 ) =  -tempDP[0];
                outjacobian ( 0,rank+5 ) =  -tempDP[1];
                outjacobian ( 1,rank+5 ) =  tempDP[0];
                break;
        }
    }

    for ( i=offset;i<robotRoot2StartJoint.size();i++ )
    {
        aJoint = ( JointPrivate * ) robotRoot2StartJoint[i];
        aBody=  aJoint->linkedDBody();
        if ( includeFreeFlyer )
            rank = aJoint->rankInConfiguration() +outOffset;
        else
            rank = aJoint->rankInConfiguration()-6+outOffset;

        tempDP = tempP - aBody->p;

        switch ( aJoint->type() )
        {
            case JointPrivate::REVOLUTE_JOINT:
                MAL_S3_VECTOR_CROSS_PRODUCT ( tempLV,aBody->w_a,tempDP );
                for ( j=0;j<3;j++ )
                    outjacobian ( j,rank ) = -tempLV[j];
                break;
            case JointPrivate::PRISMATIC_JOINT:
                for ( j=0;j<3;j++ )
                    outjacobian ( j,rank ) = -aBody->w_a[j];
                break;
            case JointPrivate::FREE_JOINT:
                for ( j=0;j<3;j++ )
                    outjacobian ( j,rank+j ) = -1.0;

                outjacobian ( 1,rank+3 ) =  tempDP[2];
                outjacobian ( 2,rank+3 ) =  -tempDP[1];
                outjacobian ( 0,rank+4 ) =  -tempDP[2];
                outjacobian ( 2,rank+4 ) =  tempDP[0];
                outjacobian ( 0,rank+5 ) =  tempDP[1];
                outjacobian ( 1,rank+5 ) =  -tempDP[0];
                break;
        }
    }
    if ( includeFreeFlyer )
    {
        tempDP = tempP - StartJoint->linkedDBody()->p;

        unsigned k = outOffset;
        outjacobian(0,k) = 1.0; outjacobian(1,k) = 0.0; outjacobian(2,k) = 0.0; k++;
        outjacobian(0,k) = 0.0; outjacobian(1,k) = 1.0; outjacobian(2,k) = 0.0; k++;
        outjacobian(0,k) = 0.0; outjacobian(1,k) = 0.0; outjacobian(2,k) = 1.0; k++;

        outjacobian ( 0,k ) =  0.0; outjacobian ( 1,k ) = -tempDP[2]; outjacobian ( 2,k ) = tempDP[1];k++;
        outjacobian ( 0,k ) = tempDP[2]; outjacobian ( 1,k ) =  0.0; outjacobian ( 2,k ) = -tempDP[0];k++;
        outjacobian ( 0,k ) =  -tempDP[1]; outjacobian ( 1,k ) =  tempDP[0]; outjacobian ( 2,k ) =  0.0;

    }
    return true;
}

bool DynMultiBodyPrivate::getOrientationJacobian ( const CjrlJoint& inStartJoint,
        const CjrlJoint& inEndJoint,
        matrixNxP& outjacobian,
        unsigned int outOffset,
        bool includeFreeFlyer )
{
    unsigned int valNumberDof = ( includeFreeFlyer==true ) ?numberDof() :numberDof()-6;
    unsigned int lengthJacobian = MAL_MATRIX_NB_COLS ( outjacobian );
    if ( ( MAL_MATRIX_NB_ROWS ( outjacobian ) != 3 ) || ( lengthJacobian < valNumberDof + outOffset ) )
        return false;

    unsigned int i,j;

    //This function should not erase the contents of argument outjacobian
    //MAL_MATRIX_FILL(outjacobian,0.0);

    //determine participating joints
    std::vector<CjrlJoint *> robotRoot2StartJoint, robotRoot2EndJoint;
    JointPrivate* StartJoint = ( JointPrivate* ) ( &inStartJoint );
    JointPrivate* EndJoint = ( JointPrivate* ) ( &inEndJoint );
    robotRoot2StartJoint = StartJoint->jointsFromRootToThis();
    robotRoot2EndJoint = EndJoint->jointsFromRootToThis();

    unsigned int offset = 1;
    unsigned int minChain = ( robotRoot2StartJoint.size() <robotRoot2EndJoint.size() )
                            ?robotRoot2StartJoint.size() :robotRoot2EndJoint.size();

    for ( i=1; i< minChain; i++ )
    {
        if ( ( robotRoot2StartJoint[i]==robotRoot2EndJoint[i] ) )
            offset++;
    }

    unsigned int rank;
    JointPrivate* aJoint;
    DynamicBodyPrivate* aBody;

    for ( i=offset;i<robotRoot2EndJoint.size();i++ )
    {
        aJoint= ( JointPrivate * ) robotRoot2EndJoint[i];
        aBody=  aJoint->linkedDBody();
        if ( includeFreeFlyer )
            rank = aJoint->rankInConfiguration() +outOffset;
        else
            rank = aJoint->rankInConfiguration()-6+outOffset;

        switch ( aJoint->type() )
        {
            case JointPrivate::REVOLUTE_JOINT:
                for ( j=0;j<3;j++ )
                    outjacobian ( j,rank ) = aBody->w_a[j];

                break;
            case JointPrivate::PRISMATIC_JOINT:
                break;
            case JointPrivate::FREE_JOINT:
                for ( j=0;j<3;j++ )
                    outjacobian ( j,rank+j ) =  1.0;
                break;
        }
    }

    for ( i=offset;i<robotRoot2StartJoint.size();i++ )
    {
        aJoint = ( JointPrivate * ) robotRoot2StartJoint[i];
        aBody=  aJoint->linkedDBody();
        if ( includeFreeFlyer )
            rank = aJoint->rankInConfiguration() +outOffset;
        else
            rank = aJoint->rankInConfiguration()-6+outOffset;

        tempDP = tempP - aBody->p;

        switch ( aJoint->type() )
        {
            case JointPrivate::REVOLUTE_JOINT:
                for ( j=0;j<3;j++ )
                    outjacobian ( j,rank ) = -aBody->w_a[j];
                break;
            case JointPrivate::PRISMATIC_JOINT:
                break;
            case JointPrivate::FREE_JOINT:
                for ( j=0;j<3;j++ )
                    outjacobian ( j,rank+j ) =  -1.0;
                break;
        }
    }

    if ( includeFreeFlyer )
    {
        for (unsigned int ii=0;ii<3;ii++)
            for (unsigned int jj=outOffset;jj<outOffset+3;jj++)
                outjacobian(ii,jj) = 0.0;
        
        unsigned k = outOffset+3;
        outjacobian(0,k) = 1.0; outjacobian(1,k) = 0.0; outjacobian(2,k) = 0.0; k++;
        outjacobian(0,k) = 0.0; outjacobian(1,k) = 1.0; outjacobian(2,k) = 0.0; k++;
        outjacobian(0,k) = 0.0; outjacobian(1,k) = 0.0; outjacobian(2,k) = 1.0;
    }
    return true;
}
