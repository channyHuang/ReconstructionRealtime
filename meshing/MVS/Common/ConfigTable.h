////////////////////////////////////////////////////////////////////
// ConfigTable.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_CONFIGTABLE_H__
#define __SEACAVE_CONFIGTABLE_H__


// I N C L U D E S /////////////////////////////////////////////////

#include "SML.h"


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

typedef struct CFGITEM_TYPE {
	enum {
		NA = 0, // no special states available
		TEMP = (1 << 0), // this item lives only this instance and it will not be save
	};
	Flags state;
	std::string name;
	std::string desc;
	std::string defval;
	StringArr vals;
} CFGITEM;


// P R O T O T Y P E S /////////////////////////////////////////////

/**
 * Configuration table interface.
 */

class GENERAL_API CConfigTable
{
public:
	CConfigTable(const std::string&);
	~CConfigTable();

	void			Release();

	// main methods
				 void		Insert(const std::string&);
				 void		Remove(const std::string&);
		  const  SML&		GetConfig(const std::string&) const;
				 SML&		GetConfig(const std::string&);
		  const  SML&		GetConfig() const						{ return m_oSML; }
				 SML&		GetConfig()								{ return m_oSML; }
	inline	SMLVALUE&		operator[] (const std::string& name)			{ return m_oSML[name]; }
	inline		 IDX		InsertChild(CConfigTable& oCfg)			{ oCfg.SetParent(this); return m_oSML.InsertChild((const LPSML)&oCfg.m_oSML); }
	inline		 void		RemoveChild(CConfigTable& oCfg)			{ oCfg.SetParent(NULL); m_oSML.RemoveChild(oCfg.m_oSML.GetName()); }

	// misc methods
				 bool		Load(const std::string&);
				 bool		Load(ISTREAM&);
				 bool		Save(const std::string&, SML::SAVEFLAG=SML::NONE) const;
				 bool		Save(OSTREAM&, SML::SAVEFLAG=SML::NONE) const;
	inline const std::string&	GetName() const							{ return m_oSML.GetName(); }
	inline CConfigTable*	GetParent() const						{ return m_pParent; }
	inline void				SetParent(CConfigTable* pParent)		{ m_pParent = pParent; }
	static void STCALL		ItemInitData(const std::string&, SMLVALUE&, void*);
	static bool STCALL		ItemSaveData(const SMLVALUE&, void*);
	static void STCALL		ItemReleaseData(SMLVALUE&, void*);

private:
	SML				m_oSML;
	CConfigTable*	m_pParent;
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_CONFIGTABLE_H__
