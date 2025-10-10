#ifndef exhaustLib
#define exhaustLib



class exhaustLib {
	public:
	#ifdef DIESEL
	void dieselPuffs(bool engineOn);
	#endif
	#ifdef STEAM
	void steamPuffs(bool engineOn);
	#endif
	private:
	bool engineIdle;
	#ifdef DIESEL
	bool firstIgnition;
	bool ignitionComplete;
	#endif
	#ifdef STEAM
	bool doPuff;
	bool donePuff;
	#endif

};
#endif