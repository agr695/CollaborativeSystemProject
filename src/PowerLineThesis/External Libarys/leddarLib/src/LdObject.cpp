// *****************************************************************************
// Module..: Leddar
//
/// \file    LdObject.cpp
///
/// \brief   Base class of all LeddarTech SDK objects.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
//
//
// Copyright (c) 2016 LeddarTech Inc. All rights reserved.
// *****************************************************************************

#include "LdObject.h"

// *****************************************************************************
// Function: LdObject::LdObject
//
/// \brief   Constructor.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

LeddarCore::LdObject::LdObject( void )
{
}

// *****************************************************************************
// Function: LdObject::~LdObject
//
/// \brief   Destructor.
///          This object need to be disconnected to from all connected object.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

LeddarCore::LdObject::~LdObject( void )
{
    DisconnectAll();
}

// *****************************************************************************
// Function: LdObject::ConnectSignal
//
/// \brief   Connect to the sender's signal.
///
/// \param aSender Pointer to object to connect.
/// \param aSignal Signal to connect.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdObject::ConnectSignal( LdObject *aSender, const SIGNALS aSignal )
{

    mReceiverMap.insert( std::make_pair( aSender, aSignal ) );

    aSender->mConnectedObject.push_back( this );
}

// *****************************************************************************
// Function: LdObject::DisconnectSignal
//
/// \brief   Disconnect to the sender's signal.
///
/// \param aSender Pointer to the object to disconnect.
/// \param aSignal Signal to disconnect.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdObject::DisconnectSignal( LdObject *aSender, const SIGNALS aSignal )
{
    std::pair<std::multimap< LdObject *, SIGNALS>::iterator, std::multimap< LdObject *, SIGNALS>::iterator> lRange = mReceiverMap.equal_range( aSender );

    if( lRange.first == mReceiverMap.end() )
    {
        return;
    }

    for( std::multimap< LdObject *, SIGNALS>::iterator lIter = lRange.first; lIter != lRange.second; ++lIter )
    {
        if( lIter->second == aSignal && lIter->first == aSender )
        {
            std::multimap< LdObject *, SIGNALS>::iterator lIterToErase = lIter;
            ++lIter;
            mReceiverMap.erase( lIterToErase );

            if( lIter == mReceiverMap.end() )
            {
                break;
            }
        }
    }

    // If there is no aSender object, we need to remove this object in the mConnectedObject
    if( mReceiverMap.find( aSender ) == mReceiverMap.end() )
    {
        for( std::list< LdObject * >::iterator lConnectedIter = aSender->mConnectedObject.begin(); lConnectedIter != aSender->mConnectedObject.end(); )
        {
            if( ( *lConnectedIter ) == this )
            {
                lConnectedIter = aSender->mConnectedObject.erase( lConnectedIter );
            }
            else
            {
                ++lConnectedIter;
            }
        }
    }
}

// *****************************************************************************
// Function: LdObject::DisconnectAll
//
/// \brief   Disconnect this object to all senders connected.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdObject::DisconnectAll( void )
{
    // Delete links between the receiver and this object
    for( std::multimap< LdObject *, SIGNALS>::iterator lIter = mReceiverMap.begin(); lIter != mReceiverMap.end(); ++lIter )
    {
        for( std::list< LdObject * >::iterator lConnectedIter = ( *lIter ).first->mConnectedObject.begin(); lConnectedIter != ( *lIter ).first->mConnectedObject.end(); )
        {
            if( ( *lConnectedIter ) == this )
            {
                lConnectedIter = ( *lIter ).first->mConnectedObject.erase( lConnectedIter );
            }
            else
            {
                ++lConnectedIter;
            }
        }
    }

    // Delete links between this object and receiver
    for( std::list< LdObject * >::iterator lIter = mConnectedObject.begin(); lIter != mConnectedObject.end(); ++lIter )
    {
        ( *lIter )->mReceiverMap.erase( this );
    }
}

// *****************************************************************************
// Function: LdObject::EmitSignal
//
/// \brief   Notify all connected object.
///
/// \param   aSignal Notification signal.
///
/// \author  Patrick Boulay
///
/// \since   January 2016
// *****************************************************************************

void
LeddarCore::LdObject::EmitSignal( const SIGNALS aSignal, void *aExtraData )
{
    for( std::multimap< LdObject *, SIGNALS>::iterator lIter = mReceiverMap.begin(); lIter != mReceiverMap.end(); ++lIter )
    {
        if( lIter->second == aSignal )
        {
            lIter->first->Callback( this, lIter->second, aExtraData );
        }
    }
}

