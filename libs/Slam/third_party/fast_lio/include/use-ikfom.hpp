#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2; 
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

struct state_ikfom {
	typedef state_ikfom self;
	typedef double scalar;
	enum {DOF = 23, DIM = 24};

	std::vector<std::pair<int, int> > S2_state;
	std::vector<std::pair<int, int> > SO3_state;
	std::vector<std::pair<std::pair<int, int>, int> > vect_state;

	MTK::SubManifold<vect3, 0, 0> pos;
	MTK::SubManifold<SO3, 3, 3> rot;
	MTK::SubManifold<SO3, 6, 6> offset_R_L_I;
	MTK::SubManifold<vect3, 9, 9> offset_T_L_I;
	MTK::SubManifold<vect3, 12, 12> vel;
	MTK::SubManifold<vect3, 15, 15> bg;
	MTK::SubManifold<vect3, 18, 18> ba;
	MTK::SubManifold<S2, 21, 21> grav;

	state_ikfom(const vect3& pos = vect3(),
	            const SO3& rot = SO3(),
	            const SO3& offset_R_L_I = SO3(),
	            const vect3& offset_T_L_I = vect3(),
	            const vect3& vel = vect3(),
	            const vect3& bg = vect3(),
	            const vect3& ba = vect3(),
	            const S2& grav = S2())
	    : pos(pos),
	      rot(rot),
	      offset_R_L_I(offset_R_L_I),
	      offset_T_L_I(offset_T_L_I),
	      vel(vel),
	      bg(bg),
	      ba(ba),
	      grav(grav)
	{}

	int getDOF() const { return DOF; }

	void boxplus(const MTK::vectview<const scalar, DOF>& __vec, scalar __scale = 1) {
		pos.boxplus(MTK::subvector(__vec, &self::pos), __scale);
		rot.boxplus(MTK::subvector(__vec, &self::rot), __scale);
		offset_R_L_I.boxplus(MTK::subvector(__vec, &self::offset_R_L_I), __scale);
		offset_T_L_I.boxplus(MTK::subvector(__vec, &self::offset_T_L_I), __scale);
		vel.boxplus(MTK::subvector(__vec, &self::vel), __scale);
		bg.boxplus(MTK::subvector(__vec, &self::bg), __scale);
		ba.boxplus(MTK::subvector(__vec, &self::ba), __scale);
		grav.boxplus(MTK::subvector(__vec, &self::grav), __scale);
	}

	void oplus(const MTK::vectview<const scalar, DIM>& __vec, scalar __scale = 1) {
		pos.oplus(MTK::subvector_(__vec, &self::pos), __scale);
		rot.oplus(MTK::subvector_(__vec, &self::rot), __scale);
		offset_R_L_I.oplus(MTK::subvector_(__vec, &self::offset_R_L_I), __scale);
		offset_T_L_I.oplus(MTK::subvector_(__vec, &self::offset_T_L_I), __scale);
		vel.oplus(MTK::subvector_(__vec, &self::vel), __scale);
		bg.oplus(MTK::subvector_(__vec, &self::bg), __scale);
		ba.oplus(MTK::subvector_(__vec, &self::ba), __scale);
		grav.oplus(MTK::subvector_(__vec, &self::grav), __scale);
	}

	void boxminus(MTK::vectview<scalar, DOF> __res, const state_ikfom& __oth) const {
		pos.boxminus(MTK::subvector(__res, &self::pos), __oth.pos);
		rot.boxminus(MTK::subvector(__res, &self::rot), __oth.rot);
		offset_R_L_I.boxminus(MTK::subvector(__res, &self::offset_R_L_I), __oth.offset_R_L_I);
		offset_T_L_I.boxminus(MTK::subvector(__res, &self::offset_T_L_I), __oth.offset_T_L_I);
		vel.boxminus(MTK::subvector(__res, &self::vel), __oth.vel);
		bg.boxminus(MTK::subvector(__res, &self::bg), __oth.bg);
		ba.boxminus(MTK::subvector(__res, &self::ba), __oth.ba);
		grav.boxminus(MTK::subvector(__res, &self::grav), __oth.grav);
	}

	friend std::ostream& operator<<(std::ostream& __os, const state_ikfom& __var) {
		return __os << __var.pos << " " << __var.rot << " " << __var.offset_R_L_I << " "
		            << __var.offset_T_L_I << " " << __var.vel << " " << __var.bg << " "
		            << __var.ba << " " << __var.grav << " ";
	}

	void build_S2_state() {
		S2_state.clear();
		S2_state.push_back(std::make_pair(grav.IDX, grav.DIM));
	}

	void build_vect_state() {
		vect_state.clear();
		vect_state.push_back(std::make_pair(std::make_pair(pos.IDX, pos.DIM), vect3::DOF));
		vect_state.push_back(std::make_pair(std::make_pair(offset_T_L_I.IDX, offset_T_L_I.DIM), vect3::DOF));
		vect_state.push_back(std::make_pair(std::make_pair(vel.IDX, vel.DIM), vect3::DOF));
		vect_state.push_back(std::make_pair(std::make_pair(bg.IDX, bg.DIM), vect3::DOF));
		vect_state.push_back(std::make_pair(std::make_pair(ba.IDX, ba.DIM), vect3::DOF));
	}

	void build_SO3_state() {
		SO3_state.clear();
		SO3_state.push_back(std::make_pair(rot.IDX, rot.DIM));
		SO3_state.push_back(std::make_pair(offset_R_L_I.IDX, offset_R_L_I.DIM));
	}

	void S2_hat(Eigen::Matrix<scalar, 3, 3>& res, int idx) {
		if (grav.IDX == idx) { grav.S2_hat(res); }
	}

	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3>& res, int idx) {
		if (grav.IDX == idx) { grav.S2_Nx_yy(res); }
	}

	void S2_Mx(Eigen::Matrix<scalar, 3, 2>& res, Eigen::Matrix<scalar, 2, 1> dx, int idx) {
		if (grav.IDX == idx) { grav.S2_Mx(res, dx); }
	}

	friend std::istream& operator>>(std::istream& __is, state_ikfom& __var) {
		return __is >> __var.pos >> __var.rot >> __var.offset_R_L_I >> __var.offset_T_L_I
		            >> __var.vel >> __var.bg >> __var.ba >> __var.grav;
	}
};

struct input_ikfom {
	typedef input_ikfom self;
	typedef double scalar;
	enum {DOF = 6, DIM = 6};

	std::vector<std::pair<int, int> > S2_state;
	std::vector<std::pair<int, int> > SO3_state;
	std::vector<std::pair<std::pair<int, int>, int> > vect_state;

	MTK::SubManifold<vect3, 0, 0> acc;
	MTK::SubManifold<vect3, 3, 3> gyro;

	input_ikfom(const vect3& acc = vect3(), const vect3& gyro = vect3())
	    : acc(acc), gyro(gyro)
	{}

	int getDOF() const { return DOF; }

	void boxplus(const MTK::vectview<const scalar, DOF>& __vec, scalar __scale = 1) {
		acc.boxplus(MTK::subvector(__vec, &self::acc), __scale);
		gyro.boxplus(MTK::subvector(__vec, &self::gyro), __scale);
	}

	void oplus(const MTK::vectview<const scalar, DIM>& __vec, scalar __scale = 1) {
		acc.oplus(MTK::subvector_(__vec, &self::acc), __scale);
		gyro.oplus(MTK::subvector_(__vec, &self::gyro), __scale);
	}

	void boxminus(MTK::vectview<scalar, DOF> __res, const input_ikfom& __oth) const {
		acc.boxminus(MTK::subvector(__res, &self::acc), __oth.acc);
		gyro.boxminus(MTK::subvector(__res, &self::gyro), __oth.gyro);
	}

	friend std::ostream& operator<<(std::ostream& __os, const input_ikfom& __var) {
		return __os << __var.acc << " " << __var.gyro << " ";
	}

	void build_S2_state() { S2_state.clear(); }

	void build_vect_state() {
		vect_state.clear();
		vect_state.push_back(std::make_pair(std::make_pair(acc.IDX, acc.DIM), vect3::DOF));
		vect_state.push_back(std::make_pair(std::make_pair(gyro.IDX, gyro.DIM), vect3::DOF));
	}

	void build_SO3_state() { SO3_state.clear(); }
	void S2_hat(Eigen::Matrix<scalar, 3, 3>&, int) {}
	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3>&, int) {}
	void S2_Mx(Eigen::Matrix<scalar, 3, 2>&, Eigen::Matrix<scalar, 2, 1>, int) {}

	friend std::istream& operator>>(std::istream& __is, input_ikfom& __var) {
		return __is >> __var.acc >> __var.gyro;
	}
};

struct process_noise_ikfom {
	typedef process_noise_ikfom self;
	typedef double scalar;
	enum {DOF = 12, DIM = 12};

	std::vector<std::pair<int, int> > S2_state;
	std::vector<std::pair<int, int> > SO3_state;
	std::vector<std::pair<std::pair<int, int>, int> > vect_state;

	MTK::SubManifold<vect3, 0, 0> ng;
	MTK::SubManifold<vect3, 3, 3> na;
	MTK::SubManifold<vect3, 6, 6> nbg;
	MTK::SubManifold<vect3, 9, 9> nba;

	process_noise_ikfom(const vect3& ng = vect3(),
	                    const vect3& na = vect3(),
	                    const vect3& nbg = vect3(),
	                    const vect3& nba = vect3())
	    : ng(ng), na(na), nbg(nbg), nba(nba)
	{}

	int getDOF() const { return DOF; }

	void boxplus(const MTK::vectview<const scalar, DOF>& __vec, scalar __scale = 1) {
		ng.boxplus(MTK::subvector(__vec, &self::ng), __scale);
		na.boxplus(MTK::subvector(__vec, &self::na), __scale);
		nbg.boxplus(MTK::subvector(__vec, &self::nbg), __scale);
		nba.boxplus(MTK::subvector(__vec, &self::nba), __scale);
	}

	void oplus(const MTK::vectview<const scalar, DIM>& __vec, scalar __scale = 1) {
		ng.oplus(MTK::subvector_(__vec, &self::ng), __scale);
		na.oplus(MTK::subvector_(__vec, &self::na), __scale);
		nbg.oplus(MTK::subvector_(__vec, &self::nbg), __scale);
		nba.oplus(MTK::subvector_(__vec, &self::nba), __scale);
	}

	void boxminus(MTK::vectview<scalar, DOF> __res, const process_noise_ikfom& __oth) const {
		ng.boxminus(MTK::subvector(__res, &self::ng), __oth.ng);
		na.boxminus(MTK::subvector(__res, &self::na), __oth.na);
		nbg.boxminus(MTK::subvector(__res, &self::nbg), __oth.nbg);
		nba.boxminus(MTK::subvector(__res, &self::nba), __oth.nba);
	}

	friend std::ostream& operator<<(std::ostream& __os, const process_noise_ikfom& __var) {
		return __os << __var.ng << " " << __var.na << " " << __var.nbg << " " << __var.nba << " ";
	}

	void build_S2_state() { S2_state.clear(); }

	void build_vect_state() {
		vect_state.clear();
		vect_state.push_back(std::make_pair(std::make_pair(ng.IDX, ng.DIM), vect3::DOF));
		vect_state.push_back(std::make_pair(std::make_pair(na.IDX, na.DIM), vect3::DOF));
		vect_state.push_back(std::make_pair(std::make_pair(nbg.IDX, nbg.DIM), vect3::DOF));
		vect_state.push_back(std::make_pair(std::make_pair(nba.IDX, nba.DIM), vect3::DOF));
	}

	void build_SO3_state() { SO3_state.clear(); }
	void S2_hat(Eigen::Matrix<scalar, 3, 3>&, int) {}
	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3>&, int) {}
	void S2_Mx(Eigen::Matrix<scalar, 3, 2>&, Eigen::Matrix<scalar, 2, 1>, int) {}

	friend std::istream& operator>>(std::istream& __is, process_noise_ikfom& __var) {
		return __is >> __var.ng >> __var.na >> __var.nbg >> __var.nba;
	}
};

MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
{
	MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
	MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);// 0.03
	MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001); // *dt 0.01 0.01 * dt * dt 0.05
	MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
	MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001);   //0.001 0.05 0.0001/out 0.01
	return cov;
}

//double L_offset_to_I[3] = {0.04165, 0.02326, -0.0284}; // Avia 
//vect3 Lidar_offset_to_IMU(L_offset_to_I, 3);
Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	vect3 a_inertial = s.rot * (in.acc-s.ba); 
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];
		res(i + 3) =  omega[i]; 
		res(i + 12) = a_inertial[i] + s.grav[i]; 
	}
	return res;
}

Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 23> cov = Eigen::Matrix<double, 24, 23>::Zero();
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	vect3 acc_;
	in.acc.boxminus(acc_, s.ba);
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);
	cov.template block<3, 3>(12, 18) = -s.rot.toRotationMatrix();
	Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	s.S2_Mx(grav_matrix, vec, 21);
	cov.template block<3, 2>(12, 21) =  grav_matrix; 
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity(); 
	return cov;
}


Eigen::Matrix<double, 24, 12> df_dw(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
	cov.template block<3, 3>(12, 3) = -s.rot.toRotationMatrix();
	cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
	return cov;
}

vect3 SO3ToEuler(const SO3 &orient) 
{
	Eigen::Matrix<double, 3, 1> _ang;
	Eigen::Vector4d q_data = orient.coeffs().transpose();
	//scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
	double sqw = q_data[3]*q_data[3];
	double sqx = q_data[0]*q_data[0];
	double sqy = q_data[1]*q_data[1];
	double sqz = q_data[2]*q_data[2];
	double unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
	double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

	if (test > 0.49999*unit) { // singularity at north pole
	
		_ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
	if (test < -0.49999*unit) { // singularity at south pole
		_ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
		
	_ang <<
			std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2] , -sqx - sqy + sqz + sqw),
			std::asin (2*test/unit),
			std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0] , sqx - sqy - sqz + sqw);
	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
	vect3 euler_ang(temp, 3);
		// euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
	return euler_ang;
}

#endif
