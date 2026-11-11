// Code written/implemented by:	Kristijan Kolozvari (kkolozvari@fullerton.edu)
//								California State University, Fullerton 
//								Kutay Orakcal
//								Bogazici University, Istanbul, Turkey
//								John Wallace
//								University of California, Los Angeles
//
// Created: 07/2015
//
// Description: This file contains the class implementation for 
// uniaxialMaterial ConcreteCM, which is a uniaxial hysteretic 
// constitutive model for concrete developed by Chang and Mander(1994).
// This model is a refined, rule - based, generalized, and non - dimensional 
// constitutive model that allows calibration of the monotonic and hysteretic 
// material modeling parameters, and can simulate the hysteretic behavior of 
// confined and unconfined, ordinary and high - strength concrete, in both 
// cyclic compression and tension. The model addresses important behavioral 
// features, such as continuous hysteretic behavior under cyclic compression 
// and tension, progressive stiffness degradation associated with smooth 
// unloading and reloading curves at increasing strain values, and gradual 
// crack closure effects. 
//
// References:
// 1) Chang, G.A. and Mander, J.B. (1994), “Seismic Energy Based Fatigue Damage 
// Analysis of Bridge Columns: Part I – Evaluation of Seismic Capacity”, NCEER 
// Technical Report No. NCEER-94-0006, State University of New York, Buffalo.
// 2) Kutay Orakcal (2004), "Nonlinear Modeling and Analysis of Slender Reinforced 
// Concrete Walls", PhD Dissertation, Department of Civil and Environmental Engineering, 
// University of California, Los Angeles.
//
// Source: /usr/local/cvs/OpenSees/SRC/material/uniaxial/ConcreteCM.h
//
// Rev: 1


#ifndef ConcreteCM_h
#define ConcreteCM_h
#include <UniaxialMaterial.h>

class ConcreteCM : public UniaxialMaterial
{
public:
	// Constructors
  ConcreteCM(int tag, double fpcc, double epcc, double Ec, double rc, double xcrn, double ft, double et, double rt, double xcrp, int mon=0, int Gap=0);
	ConcreteCM(int tag);
	ConcreteCM();

        const char *getClassType(void) const {return "ConcreteCM";};

	// Destructor
	~ConcreteCM();

	int setTrialStrain(double strain, double strainRate = 0.0); 
	double getStrain(void);      
	double getStress(void);
	double getTangent(void);
	double getInitialTangent(void) {return Ec;}

	Vector getInputParameters(void);

	int commitState(void);
	int revertToLastCommit(void);    
	int revertToStart(void);        

	double getCommittedStrain(void);
	double getCommittedStress(void);
	double getCommittedCyclicCrackingStrain(void);

	UniaxialMaterial *getCopy(void);

	int sendSelf(int commitTag, Channel &theChannel);  
	int recvSelf(int commitTag, Channel &theChannel, 
		FEM_ObjectBroker &theBroker);    

	Response *setResponse (const char **argv, int argc, 
		OPS_Stream &theOutputStream);
	int getResponse (int responseID, Information &matInformation);   

	void Print(OPS_Stream &s, int flag =0);

	// AddingSensitivity:BEGIN //////////////////////////////////////////
	int    setParameter             (const char **argv, int argc, Information &info);
	int    updateParameter          (int parameterID, Information &info);
	int    activateParameter        (int parameterID);
	double getStressSensitivity     (int gradNumber, bool conditional);
	int    commitSensitivity        (double strainGradient, int gradNumber, int numGrads);
	// AddingSensitivity:END ///////////////////////////////////////////

protected:

private:

	// All data members are zero-initialized here so that no constructor
	// (e.g. ConcreteCM(int tag), used by getCopy) leaves them undefined.

	// Input parameters
	double fpcc = 0.0;	// peak compressive stress
	double epcc = 0.0;	// strain at peak compressive stress	
	double Ec = 0.0;		// Young's modulus
	double rc = 0.0;		// shape parameter in compression
	double xcrn = 0.0;	// cracking strain in compression
	double ft = 0.0;		// peak tensile stress
	double et = 0.0;		// strain at peak tensile stress
	double rt = 0.0;		// shape parameter in tension
	double xcrp = 0.0;	// cracking strain in tension
	int mon = 0;		// switch for monotonic vs. cyclic (internal)
	int Gap = 0;		// switch for more versus less gradual gap closure
	int dummy = 0;		// dummy input variable

	// CONVERGED history variables
	double Ceunn = 0.0;
	double Cfunn = 0.0;
	double Ceunp = 0.0;
	double Cfunp = 0.0;
	double Cer = 0.0;
	double Cfr = 0.0;
	double Cer0n = 0.0;
	double Cfr0n = 0.0;
	double Cer0p = 0.0;
	double Cfr0p = 0.0;
	double Ce0 = 0.0;
	double Cea = 0.0;
	double Ceb = 0.0;
	double Ced = 0.0;
	double Cinc = 0.0;
	double Crule = 0.0;

	// CONVERGED State Variables
	double Cstrain = 0.0;
	double Cstress = 0.0;   
	double Ctangent = 0.0;	// Don't need Ctangent other than for revert and sendSelf/recvSelf

	// TRIAL history variables
	double Teunn = 0.0;
	double Tfunn = 0.0;
	double Teunp = 0.0;
	double Tfunp = 0.0;
	double Ter = 0.0;
	double Tfr = 0.0;
	double Ter0n = 0.0;
	double Tfr0n = 0.0;
	double Ter0p = 0.0;
	double Tfr0p = 0.0;
	double Te0 = 0.0;
	double Tea = 0.0;
	double Teb = 0.0;
	double Ted = 0.0;
	double Tinc = 0.0;
	double Trule = 0.0;

	// TRIAL state variables
	double Tstrain = 0.0;
	double Tstress = 0.0;   
	double Ttangent = 0.0;

	// Other variables
	double xn = 0.0;
	double nn = 0.0;
	double xsp = 0.0;
	double xp = 0.0;
	double np = 0.0;
	double xcrk = 0.0;
	double y = 0.0;
	double z = 0.0;

	double eunn = 0.0;
	double funn = 0.0;
	double espln = 0.0;
	double Epln = 0.0;
	double Esecn = 0.0;
	double Esectest = 0.0;
	double Esectest10 = 0.0; //KK
	double Esectest13 = 0.0; //KK
	double delen = 0.0;
	double delfn = 0.0;
	double fnewn = 0.0;
	double Enewn = 0.0;
	double esren = 0.0;
	double fren = 0.0;
	double Eren = 0.0;
	double fnewstn = 0.0;
	double Enewstn = 0.0;
	double esrestn = 0.0;
	double frestn = 0.0;
	double Erestn = 0.0;

	double eunp = 0.0;
	double funp = 0.0;
	double esplp = 0.0;
	double Eplp = 0.0;
	double Esecp = 0.0;
	double delep = 0.0;
	double delfp = 0.0;
	double fnewp = 0.0;
	double Enewp = 0.0;
	double esrep = 0.0;
	double frep = 0.0;
	double Erep = 0.0;
	double fnewstp = 0.0;
	double Enewstp = 0.0;
	double esrestp = 0.0;
	double frestp = 0.0;
	double Erestp = 0.0;

	double esi = 0.0;
	double esi10 = 0.0; //KK
	double esi13 = 0.0; //KK
	double fi = 0.0; 
	double fi10 = 0.0; //KK
	double fi13 = 0.0; //KK
	double Ei = 0.0;
	double Ei10 = 0.0; //KK
	double Ei13 = 0.0; //KK
	double esf = 0.0;
	double esf10 = 0.0; //KK
	double esf13 = 0.0; //KK
	double ff = 0.0;
	double ff10 = 0.0; //KK
	double ff13 = 0.0; //KK
	double Ef = 0.0;
	double Ef10 = 0.0; //KK
	double Ef13 = 0.0; //KK

	double R = 0.0;
	double R10 = 0.0; //KK
	double R13 = 0.0; //KK
	double A = 0.0;
	double A10 = 0.0; //KK
	double A13 = 0.0; //KK

	double fc = 0.0;
	double Et = 0.0;

	double fca = 0.0;
	double Eta = 0.0;

	double fcb = 0.0;
	double Etb = 0.0;

	double fa = 0.0;
	double fb = 0.0;

	// Member functions
	void fcEtnf(double e);
	void fcEtpf(double e, double e0);
	void fcEtpr6f(double e, double e0);
	void yf(double x, double n, double r);
	void zf(double x, double n, double r);

	void esplnf(double eunn, double funn);
	void Eplnf(double eunn);
	void Esecnf(double eunn, double funn);
	void delenf(double eunn);
	void delfnf(double eunn, double funn);
	void fnewnf(double eunn, double funn);
	void Enewnf(double eunn, double funn);
	void esrenf(double eunn);
	void freErenf(double eunn);
	void fnewstnf(double funn, double delfn, double eunn, double er0n, double espln);
	void Enewstnf(double fnewstn, double fr0n, double eunn, double er0n);
	void esrestnf(double eunn, double delen, double er0n, double espln);   
	void freErestnf(double eunn, double funn, double er0n);

	void esplpf(double eunp, double funp, double e0, double espln);
	void Eplpf(double e0, double eunp);
	void Esecpf(double e0, double eunp, double funp, double espln);
	void delepf(double eunp, double e0);
	void delfpf(double funp, double eunp, double e0);
	void fnewpf(double funp, double eunp, double e0);
	void Enewpf(double eunp, double funp, double e0, double espln);
	void esrepf(double eunp, double e0);
	void freErepf(double eunp, double e0);
	void fnewstpf(double funp, double delfp, double eunp, double er0p, double esplp, double e0);
	void Enewstpf(double fnewstp, double fr0p, double eunp, double er0p);
	void esrestpf(double eunp, double delep, double er0p, double esplp);   
	void freErestpf(double eunp, double funp, double er0p, double e0, double espln);

	void e0eunpfunpf (double e0, double eunp, double funp, double eunn, double funn);

	void r1f(double x, double n, double r);
	void r5f(double x, double n, double r);
	void r2f(double x, double n, double r);
	void r6f(double x, double n, double r);
	void r3f(double eunn, double funn, double espln, double Epln);
	void r9f(double espln, double Epln, double eunp, double fnewp, double Enewp);
	void r8f(double eunp, double fnewp, double Enewp, double esrep, double frep, double Erep);
	void r4f(double eunp, double funp, double esplp, double Eplp);
	void r10f(double esplp, double Eplp, double eunn, double fnewn, double Enewn);
	void r7f(double eunn, double fnewn, double Enewn, double esren, double fren, double Eren);
	void r12f(double er, double fr, double ea, double fca, double Eta, double A, double R);
	void r11f(double er, double fr, double eb, double fcb, double Etb, double A, double R);
	void r13f(double ed, double eunn, double fnewn, double Enewn);
	void r14f(double er, double fr, double eb);
	void r15f(double er, double fr, double ea, double fca, double Eta, double A, double R);
	void r66f(double e, double e0);
	void r77f(double e, double e0, double er0n, double fr0n, double eunn, double fnewstn, double Enewstn, double esrestn, double frestn, double Erestn);
	void r88f(double e, double e0, double er0p, double fr0p, double eunp, double fnewstp, double Enewstp, double esrestp, double frestp, double Erestp);

	void ea1112f(double eb, double espln, double esplp, double eunn, double eunp);
	void eb1112f(double ea, double espln, double esplp, double eunn, double eunp);
	void eb1415f(double ea, double fa, double Esecn);


	void RAf(double esi, double fi, double Ei, double esf, double ff, double Ef);
	void fcEturf(double es, double esi, double fi, double esf, double ff, double Ei, double Ef, double A, double R);

	// AddingSensitivity:BEGIN //////////////////////////////////////////
	int parameterID;
	Matrix *SHVs;
	// AddingSensitivity:END ///////////////////////////////////////////
};

#endif
