//
// voctree_bf.h -
// 2012 INCORL. Hanyang Univ.
//
//
#ifndef _VOCTREE_BF_H_
#define _VOCTREE_BF_H_

#include <vector>
#include <map>
#include <deque>
#include <queue>
#include <algorithm>
#include <stdio.h>

//#define show_all

#define BRANCH_FACTOR 40
#define LEVEL 3
#define DESC_DIM 72

#define PI 3.1415926535897932384626433832795

#if 1   // indoor
#define NUM_NON_CONSIDER_RECENT_FRAMES 40
#define SIGMA 1.0
#define THRESHOLD 0.7
#define NUM_CONSIDER_SEQ_LENGTH 10
#endif

#if 0   // outdoor
#define NUM_NON_CONSIDER_RECENT_FRAMES 100
#define SIGMA 0.8
#define THRESHOLD 0.8
#define NUM_CONSIDER_SEQ_LENGTH 15
#endif

#if 0   // outdoor long-loop
#define NUM_NON_CONSIDER_RECENT_FRAMES 300
#define SIGMA 0.8
#define THRESHOLD 0.5
#define NUM_CONSIDER_SEQ_LENGTH 5
#endif

//-----------------------------------------------------------------------------

template <int K, int L, int D>
class voctree_t
{
public:
	voctree_t()   { _int_node=NULL, _leaf_node=NULL; }
	~voctree_t()  { free(); }

	bool is_valid() const  { return _int_node!=NULL && _leaf_node!=NULL; }

	float trans_prob(int state, int state_prev, int numconsiderfrms, double *gausstab);

	void calc_post_prob(std::map<int,float> &lc_prob,
			std::map<int,float> &lc_prob_old,
	//		std::vector<std::pair<float,int> > &likelihood,
			std::map<int,float> &likelihood,
			int numconsiderfrms);

	bool isLoopClosing(std::vector<pair<int,float> > &top_post_prob, int *lcfrmid);

	bool load(const char *voctree_path);
	void free();

	size_t find_leaf(const float feat[D]) const;

	bool insert_doc(int doc_id, const std::vector<float*> &feat);
	//void update_voctree();  // computes weights : NO WEIGHTING

	void query_doc(const std::vector<float*> feat,
			std::vector< std::pair<float,int> > &doc_score,
			std::map<int, float> &doc_likelihood,
			int numavgwords);

	int doc_size;

	struct feat_buf {
		float buf[512][DESC_DIM];
	};

	double gausstab[512];

	vector<vector<int> > doc_forward;
	float sim_scores[1024][1024];

	std::map<int,std::vector<int> > node_connectivity;
	std::queue< std::pair<int,feat_buf> > doc_buffer;
	std::queue< int > doc_id_buffer;

protected:
	struct int_node_t { float c[D*K]; } *_int_node;
	struct leaf_node_t { std::map<int,float> doc; } *_leaf_node; // each leaf node contains indices for db having the quantized descriptor vector.
	size_t _num_int, _num_leaf;

	static size_t child_idx(size_t i, int j=0)  { return i*K+j+1; }
	static size_t parent_idx(size_t i)  { return (i+K-1)/K-1; }
	size_t leaf_idx(size_t i) const  { return i-_num_int; }
	static float dist_func(const float f0[D], const float f1[D]) {
		//	      { float r=0; for (int i=0; i<D; ++i) r+=fabs(f0[i]-f1[i]); return r; }
		float r = 1.0f;
		for (int i=0; i<D; ++i) {
			r -= (f0[i]*f1[i]);
		}
		return r;
	}
};

//-----------------------------------------------------------------------------

//template <int L> size_t _num_node(int K) {  return _num_node<L-1>(K)*K + 1; }
//template <> size_t _num_node<0>(int K)  { return 1; }

template <int K, int L, int D>
bool voctree_t<K,L,D>::load(const char *voctree_path)
{
	FILE *fp = fopen(voctree_path, "rb");
	if (fp == NULL) {
		printf("No such file: %s\n", voctree_path);
		return false;
	}
	free();
	size_t num_node = 1, num_node_level = 1;
	for (int l=1; l<L; ++l)  num_node += (num_node_level *= K);
	_num_int = num_node;  //_num_node<L-1>(K);
	_num_leaf = (num_node += (num_node_level *= K)) - _num_int; //_num_node<L>(K) - _num_int;
	printf("voctree: num_int: %d (%d), num_leaf: %d\n", (int)_num_int, (int)(_num_int*K*D*sizeof(float)), (int)_num_leaf);

	_int_node = new int_node_t[_num_int];
	_leaf_node = new leaf_node_t[_num_leaf];

	for (size_t i=0; i<_num_int; ++i)
	{
		if (fread(_int_node[i].c, sizeof(float), D*K, fp) < D*K)
		{
			fclose(fp); return false;
		}
		//    printf("internal node %d : %.7g \n", (int)i, _int_node[i].c[0]);
	}
	fclose(fp);
	//printf("done");
	//  size_t ni = child_idx(child_idx(0,49), 49);
	//  for (int i=0, ii=0; i<6; ++i) {
	//    int_node_t &node = _int_node[ni];
	//    printf("### \t");
	//    for (int j=0; j<D; ++j, ++ii)
	//      printf(" %.4g", node.c[ii]);
	//    printf(" ");
	//  }
	doc_size = 0;

//	double eta2 = 0.0;
	for(int i=0; i<512; i++) {
		int dist = i;
		gausstab[i] = 1.0/sqrt(2.0*PI*SIGMA*SIGMA)  * exp(-1.0 * ( (double)(dist*dist) / (2.0*SIGMA*SIGMA) ) );
//		eta2 += (double)gausstab[i];
	}
/*
	for(int i=0; i<512; i++) {
//		gausstab[i] = gausstab[i]/eta2*0.9;
		gausstab[i] = gausstab[i]/eta2;
//		printf("gausstab[%d] = %.7f \n", i, gausstab[i]);
	}
*/

	return true;
}


template <int K, int L, int D>
void voctree_t<K,L,D>::free()
{
	if (_int_node)  delete[] _int_node;
	if (_leaf_node)  delete[] _leaf_node;
}


template <int K, int L, int D>
size_t voctree_t<K,L,D>::find_leaf(const float feat[D]) const
{
	size_t idx = 0;  // root node
	for (int lvl=0; lvl < L; ++lvl) {
		int_node_t &node = _int_node[idx];
		float dist, mindist = dist_func(node.c, feat);
		int minidx = 0;
		for (int i=1; i < K; ++i)
			if ((dist = dist_func(&node.c[D*i], feat)) < mindist)
				mindist = dist, minidx = i;
		idx = child_idx(idx, minidx);
	}
	return idx;
}


template <int K, int L, int D>
bool voctree_t<K,L,D>::insert_doc(int doc_id, const std::vector<float*> &feat)
{
	if(doc_id != -2) {
		feat_buf fb;
		for(int i=0; i<feat.size(); i++) {
			for(int j=0; j<DESC_DIM; j++) {
				fb.buf[i][j] = feat[i][j];
			}
		}

		doc_buffer.push(pair<int,feat_buf>(feat.size(), fb));
		doc_id_buffer.push(doc_id);
	}
//	printf("\n\n  doc_buffer.size() = %d \n", doc_buffer.size());

	vector<float*> buf_feat;
	float temp[512][DESC_DIM];
	if(doc_buffer.size() > NUM_NON_CONSIDER_RECENT_FRAMES || ( doc_id == -2 && doc_buffer.size() > 0 ) ) {
//		printf(" doc_id = %d ", doc_id);
//		doc_id = doc_id - NUM_NON_CONSIDER_RECENT_FRAMES;
		doc_id = doc_id_buffer.front();
//		printf(" doc_id = %d\n", doc_id);
		for(int i=0; i<doc_buffer.front().first; i++) {
			for(int j=0; j<DESC_DIM; j++) {
				temp[i][j] = doc_buffer.front().second.buf[i][j];
			}
			buf_feat.push_back(temp[i]);
		}
//		printf("\n%d %.7f %.7f\n",doc_buffer.front().first,doc_buffer.front().second.buf[0][71], feat[0][71]);
//		for( int i=0; i<buffsize; i++) {
//			printf("\n%d %.7f %.7f\n",doc_buffer.front().first,doc_buffer.front().second.buf[0][0], feat[0][0]);
//			doc_buffer.pop_front();
//		}
		doc_buffer.pop();
		doc_id_buffer.pop();
	}
	else
		return true;

	int featcnt = 0;
	for (size_t i=0; i<buf_feat.size(); ++i)
		if (buf_feat[i] != NULL)
			++featcnt;
	if (featcnt <= 0) {
		return false;
	}

	const float w = 1.f/(float)featcnt;
	vector<int> leafidx;
	for (size_t i=0; i<buf_feat.size(); ++i) {
		if (buf_feat[i] == NULL)
			continue;
		//		printf("%.7f ", feat[i][0]);
		size_t idx = find_leaf(buf_feat[i]);
		//		printf("%.7f ", feat.at(i)[0]);
		//		size_t idx = find_leaf(feat.at(i));
		//		printf("%d ", idx);
		leafidx.push_back(leaf_idx(idx));
		leaf_node_t &leaf = _leaf_node[leaf_idx(idx)];
		std::map<int,float>::iterator it = leaf.doc.find(doc_id);
		if (it == leaf.doc.end())
			leaf.doc.insert(std::pair<int,float>(doc_id,w));
		//			leaf.doc.insert(std::pair<int,float>(doc_id, 1.0f));
		else {
			it->second += w;
			//			it->second += 1.0f;
			//      printf("%.5f\n",it->second);
		}
		//		printf(" [%d,%.5f]", (int)idx, leaf.doc[doc_id]);
	}
/*
	doc_forward.push_back(leafidx);
//	vector<vector<int> >::iterator i;
	float score_sum = 0.0f;
	float mean = 0.0f;
	float sq_score_sum = 0.0f;

	std::map<size_t,int> d2;
	int featcnt2 = 0;
	for (int l=0; l<leafidx.size(); ++l) {
		int idx=leafidx.at(l);
		std::map<size_t,int>::iterator it = d2.find(idx);
		if (it == d2.end())
			d2.insert(std::pair<size_t,int>(idx,1));
		else {
			++it->second;
			//    	  printf("ouch!!\n");
			//  printf_(" %d", (int)idx);
		}
		++featcnt2;
	}

	for(int i = 0; i < doc_forward.size(); i++) {
		std::map<size_t,int> d1;
		int featcnt1 = 0;
		for (int l=0; l<doc_forward[i].size(); ++l) {
			int idx=doc_forward[i].at(l);
			std::map<size_t,int>::iterator it = d1.find(idx);
			if (it == d1.end())
				d1.insert(std::pair<size_t,int>(idx,1));
			else {
				++it->second;
				//    	  printf("ouch!!\n");
			}
			//  printf_(" %d", (int)idx);
			++featcnt1;
		}
		std::map<size_t,int>::iterator d1_it;
		std::map<size_t,int>::const_iterator cit;
		float sim_score = 0.0f;
		for(d1_it=d1.begin(); d1_it != d1.end(); d1_it++) {
//			float n = (float)d1_it->second / (float)featcnt1;
			float n = (float)d1_it->second;
			if( (cit=d2.find(d1_it->first)) != d2.end() ) {
//				float m = (float)cit->second / (float)featcnt2;
				float m = (float)cit->second;
				sim_score += fabs(n-m)-n-m;
//				sim_score += n;
			}
		}

//		int rptcnt = 0;
//		for(int j = 0; j < doc_forward[i].size(); j++) {
//			for(int k = 0; k < leafidx.size(); k++) {
//				if(doc_forward[i].at(j) == leafidx.at(k))
//					rptcnt++;
//			}
//		}

//		printf("\n\nouch!!\n\n");
//		float sim_score = (float)rptcnt/( doc_forward[i].size() + (float)leafidx.size() );
//		sq_score_sum += sim_score*sim_score;
//		score_sum += sim_score;
//		sim_scores[i][doc_id] = 2.0f+sim_score;
		sim_scores[i][doc_id] = -sim_score;
		sim_scores[doc_id][i] = -sim_score;
//		sim_scores[i][doc_id] = 1.0f/(3.0f+sim_score);
//		sim_scores[doc_id][i] = 1.0f/(3.0f+sim_score);
//		printf("sim_scores[%d][%d] = %.7f \n", i, doc_id, 2.0f+sim_score);
	}

	mean = score_sum/(float)(doc_id+1);
	sq_score_sum /= (float)(doc_id+1);
	float stddev = sq_score_sum - mean*mean;
	stddev = sqrt(stddev);
	score_sum = 0.0f;
	for(int i = 0; i < doc_forward.size(); i++) {
		float score = sim_scores[i][doc_id];
		if(score > mean+stddev) {
//		if(score > mean) {
		sim_scores[i][doc_id] = (score-stddev)/mean * gausstab[abs(i-doc_id)];
//		sim_scores[i][doc_id] = score / mean * gausstab[abs(i-doc_id)];
//		score_sum += score/mean * gausstab[abs(i-doc_id)];
		score_sum += (score-stddev)/mean * gausstab[abs(i-doc_id)];
		}
		else {
			sim_scores[i][doc_id] = 1.0f * gausstab[abs(i-doc_id)];
			score_sum += 1.0f * gausstab[abs(i-doc_id)];
		}
	}
	for(int i = 0; i < doc_forward.size(); i++) {
		sim_scores[i][doc_id] = sim_scores[i][doc_id] / score_sum * 0.9f;
//		printf("sim_score between %d and %d = %.7f\n",i,doc_id,sim_scores[i][doc_id]);
	}

//	sim_scores[0][0] = 0.1f;

//	for(int i=0; i<doc_forward.size(); i++) {
//		for(int j=0; j<doc_forward[i].size(); j++) {
//			printf("%d ", doc_forward[i].at(j));
//		}
//		printf("\n");
//	}

	//	printf(": %d", feat.size());
	//	printf(": %d", doc_id);
*/
	doc_size++;
	return true;
}


template <int K, int L, int D>
void voctree_t<K,L,D>::query_doc(const std::vector<float*> feat,
		std::vector< std::pair<float,int> > &doc_score,
		std::map<int, float> &doc_likelihood,
		int numavgwords)
{
 	bool virtualdocremoved = false;
	std::vector<std::pair<int,int> > wordrptcnt;
	for(int i=0; i<_num_leaf; i++) {
		leaf_node_t &leaf = _leaf_node[i];
		if(leaf.doc.size() == 0) continue;
		std::map<int,float>::iterator jt;
		if( ( jt = leaf.doc.find(-1)) != leaf.doc.end() ) {
			leaf.doc.erase(jt);
			virtualdocremoved = true;
		}
		wordrptcnt.push_back(pair<int,int>(-1*leaf.doc.size(),i));
	}
	if(virtualdocremoved == true) doc_size--;
	if(wordrptcnt.size() > numavgwords) {
		std::sort(wordrptcnt.begin(), wordrptcnt.end());
		std::vector<pair<int,int> >::iterator it = wordrptcnt.begin();
		const float w2 = 1.0f / numavgwords;
		for(int i=0; i<numavgwords; i++) {
			int rptwordidx = it->second;
			it++;
			leaf_node_t &leaf = _leaf_node[rptwordidx];
			std::map<int,float>::iterator jt = leaf.doc.find(-1);
			if (jt == leaf.doc.end()) {
				leaf.doc.insert(std::pair<int,float>(-1,w2));
			}
			else {
				jt->second += w2;
			}
		}
		doc_size++;
	}

	std::map<size_t,int> q;
	int featcnt = 0;
	for (size_t i=0; i<feat.size(); ++i) {
		if (feat[i] == NULL)
			continue;
		size_t idx = find_leaf(feat[i]);
		const leaf_node_t &leaf = _leaf_node[leaf_idx(idx)];
		if (leaf.doc.size() > 0) {
			std::map<size_t,int>::iterator it = q.find(idx);
			if (it == q.end())
				q.insert(std::pair<size_t,int>(idx,1));
			else {
				++it->second;
				//    	  printf("ouch!!\n");
			}
			//  printf_(" %d", (int)idx);
		}
		++featcnt;
	}
	//for (std::map<size_t,int>::iterator i=q.begin(); i!=q.end(); ++i)
	//  printf_(" %d,%d", (int)i->first, i->second);
	//printf(": %d", q.size());
	std::map<size_t,int>::const_iterator q_it;
	std::map<int,float> score;  // doc_id, score

	double sum = 0.0;
	double scoresqsum = 0.0;
	double mean = 0.0;
	double stddev = 0.0;
	int q_res_cnt = 1;

	for (q_it = q.begin(); q_it != q.end(); ++q_it) {
		const leaf_node_t &leaf = _leaf_node[leaf_idx(q_it->first)];
		float n = q_it->second / (float)featcnt;  // query_doc's feat count

		std::map<int,float>::const_iterator d_it;
		std::map<int,float>::iterator j;
		//		printf("leaf.doc.size() = %d \n", leaf.doc.size());
//				printf("doc_size = %d\n", doc_size);
		for (d_it = leaf.doc.begin(); d_it != leaf.doc.end(); ++d_it) {
			int doc_id = d_it->first;
//			float m = d_it->second;
			n *= log10( (float)(doc_size) /(float)leaf.doc.size());
			float m = d_it->second*log10( (float)(doc_size) /(float)leaf.doc.size());
			float l1dist = -1.0f*(fabs(n-m)-n-m);
			//printf("\t%d, %.5f / %d, %.5f", q_it->first, n, doc_id, m);
//			float tfidf = m*log10( (float)(doc_size) /(float)leaf.doc.size());
//			printf("n = %.7f tfidf = %.7f\n", n,tfidf);
			if ((j = score.find(d_it->first)) == score.end()) {
//				score.insert(std::pair<int,float>(doc_id, -fabs(n-m)+n+m));
//				sum+=-fabs(n-m)+n+m;
				score.insert(std::pair<int,float>(doc_id, l1dist));
				sum+=l1dist;
//				score.insert(std::pair<int,float>(doc_id, tfidf));
//				sum+=tfidf;
				q_res_cnt++;
			}
			else {
//				j->second += -fabs(n-m)+n+m;
//				sum += -fabs(n-m)+n+m;
				j->second += l1dist;
				sum += l1dist;
//				j->second += tfidf;
//				sum+=tfidf;
			}
		}
	}

	mean = sum / (double)q_res_cnt;
	sum = 0.0;
#ifdef show_all
	printf("\nscore mean = %.7f \n", mean);
#endif

	for(int i=-1; i<doc_size-1; i++) {
		std::map<int,float>::iterator j=score.find(i);
		if(j==score.end()) {
			score.insert(pair<int,float>(i,mean));
		}
	}

 	for (std::map<int,float>::iterator j=score.begin(); j != score.end(); ++j) {
		//		doc_score.push_back(std::pair<float,int>(2.f+j->second, j->first));
//		j->second *= -1.0f;
		scoresqsum += j->second * j->second;
		sum += j->second;
	}
	scoresqsum /= score.size();
	mean = sum / score.size();
	stddev = scoresqsum - mean*mean;
	stddev = sqrt(stddev);

//	printf("\n\nscoresqsum = %.7f mean = %.7f stddev = %.7f \n\n", scoresqsum, mean, stddev);

	doc_score.clear();
	doc_score.reserve(score.size());
	doc_likelihood.clear();
//	doc_likelihood.reserve(score.size());
	for (std::map<int,float>::iterator j=score.begin(); j != score.end(); ++j) {
		//		doc_score.push_back(std::pair<float,int>(2.f+j->second, j->first));
		doc_score.push_back(std::pair<float,int>(j->second, j->first));
		if(j->second > mean+2.0f*stddev) {
//			printf("j->second = %.7f mean = %.7f stddev = %.7f \n", j->second, mean, stddev);
			doc_likelihood.insert(std::pair<int,float>(j->first, (j->second - 2.0f*stddev) / mean) );
//			doc_likelihood.insert(std::pair<int,float>(j->first, j->second  ) );
		}
		else {
//			doc_likelihood.insert(std::pair<int,float>(j->first, DEFAULT_LIKELIHOOD) );
			doc_likelihood.insert(std::pair<int,float>(j->first, 1.0f) );
//			doc_likelihood.insert(std::pair<int,float>(j->first, 0.04f));
//			doc_likelihood.insert(std::pair<int,float>(j->first, 0.1f));
//		doc_likelihood.insert(std::pair<int,float>(j->first, (float)( j->second / sum ) ) );
		}
//		doc_likelihood.insert(std::pair<int,float>(j->first, j->second ) );
//		printf("%d's  likelihood = %.7f\n", j->first, j->second);
	}
/*
	std::sort(doc_score.begin(), doc_score.end());
	int k=0;
	for (std::vector<pair<float,int> >::iterator j=doc_score.end()-1; j != doc_score.begin()-1; j--) {
		if(k<5 && j->first > mean) {
//			printf("%d's  likelihood = %.7f\n", j->second, j->first);
//			doc_likelihood.insert(std::pair<int,float>(j->second, (float)( j->first / mean) ) );
//			doc_likelihood.insert(std::pair<int,float>(j->second, (float)( fabs(j->first-stddev) / mean) ) );
			doc_likelihood.insert(std::pair<int,float>(j->second, j->first ) );
		}
		else {
//			doc_likelihood.insert(std::pair<int,float>(j->second, 1.0f ) );
			doc_likelihood.insert(std::pair<int,float>(j->second, mean ) );
//			doc_likelihood.insert(std::pair<int,float>(j->second, j->first ) );
		}
		k++;
//		printf("%d's  likelihood = %.7f\n", j->second, j->first);
	}
*/
	std::sort(doc_score.begin(), doc_score.end());
#ifdef show_all
	printf("doc_size = %d doc_score.size() = %d doc_likelihood.size() = %d\n", doc_size, doc_score.size(), doc_likelihood.size());
#endif
}

template <int K, int L, int D>
float voctree_t<K,L,D>::trans_prob(int state, int state_prev, int numconsiderfrms, double* gausstab) {
	float prob = 0.0f;
	int dist = abs(state_prev-state);

	if(state_prev == -1) {
		if(state == -1)
			prob = 0.9f;
		else
			prob = 0.1f/(float)numconsiderfrms;
	}
	else {
		if(state == -1) {
			prob = 0.1f;
		}
		else {
//			prob = sim_scores[min(state,state_prev)][max(state, state_prev)];
//			printf("trans_prob between %d and %d = %.7f\n", min(state,state_prev), max(state,state_prev), prob);
			if(dist < 10)
				prob = gausstab[dist];
//				prob = sim_scores[min(state,state_prev)][max(state, state_prev)];
			else {
				prob = 0.0f;
			}
		}
	}
	return prob;
}

template <int K, int L, int D>
void voctree_t<K,L,D>::calc_post_prob(std::map<int,float> &lc_prob,
		std::map<int,float> &lc_prob_old,
		std::map<int,float> &likelihood,
		int numconsiderfrms) {

	lc_prob.clear();
	std::map<int,float>::iterator j_map;

	int k=0;
	double eta = 0.0;
	float lhood = 0.0f;
	float belief = 0.0f;
	float postprob_prev = 0.0f;
	double trans_prob_sum = 0.0;

//	printf("likelihood.size() = %d numconsiderfrms = %d \n",likelihood.size(), numconsiderfrms);

/*
	float t_p[likelihood.size()-1][numconsiderfrms];
	for(int j=0; j<numconsiderfrms; j++) {
		std::map<int,float>::iterator it;
		float score_sum = 0.0f;
		for(it = likelihood.begin(); it != likelihood.end(); ++it) {
			if(it->first == -1)
				continue;
			float crained_gauss = gausstab[abs(it->first-j)] > 0.01 ? gausstab[abs(it->first-j)] : 0.01;
			t_p[it->first][j] = sim_scores[it->first][j]*crained_gauss;
//			t_p[it->first][j] = sim_scores[it->first][j]*gausstab[abs(it->first-j)];
//			t_p[it->first][j] = gausstab[abs(it->first-j)];
			score_sum += t_p[it->first][j];
		}
		for(it = likelihood.begin(); it != likelihood.end(); ++it) {
			if(it->first == -1)
				continue;
			t_p[it->first][j] = t_p[it->first][j] / score_sum * 0.9f;
		}
	}
*/

/*	float t_p[likelihood.size()-1][numconsiderfrms];
	for(int j=0; j<numconsiderfrms; j++) {
		std::map<int,float>::iterator it;
		float score_sum = 0.0f;
		for(it = likelihood.begin(); it != likelihood.end(); ++it) {
			if(it->first == -1)
				continue;
//			float crained_gauss = gausstab[abs(it->first-j)] > 0.01 ? gausstab[abs(it->first-j)] : 0.01;
//			t_p[it->first][j] = sim_scores[it->first][j]*crained_gauss;
//			t_p[it->first][j] = sim_scores[it->first][j]*gausstab[abs(it->first-j)];
			t_p[it->first][j] = gausstab[abs(it->first-j)];
			score_sum += t_p[it->first][j];
		}
		for(it = likelihood.begin(); it != likelihood.end(); ++it) {
			if(it->first == -1)
				continue;
			t_p[it->first][j] = t_p[it->first][j] / score_sum * 0.9f;
		}
	}*/ // gauss normalization to 0.9

	std::map<int,float>::iterator it;
	for(it = likelihood.begin(); it != likelihood.end(); ++it) {
//		if(k>numconsiderfrms) break;
		lhood = it->second;
		belief = 0.0f;
		postprob_prev = 0.0f;
		trans_prob_sum = 0.0f;
//		printf("it->first = %d it->second = %.7f\n", it->first, it->second);

		for(int j=-1; j<numconsiderfrms; j++) {
//		for(int j=0; j<numconsiderfrms; j++) {
			if( (j_map = lc_prob_old.find(j)) == lc_prob_old.end()) {
//				postprob_prev = 1.0f / (float)(lc_prob_old.size()+1);
//				cout << "ouch!!!  " << j << endl;
				postprob_prev = 0.0f;
			}
			else {
				//			j_map = lc_prob_old.find(j);
				postprob_prev = j_map->second;
//				printf("j_map->first = %d j_map->second = %.7f\n",j_map->first, j_map->second);
			}
			belief += trans_prob(it->first, j, numconsiderfrms, gausstab)*postprob_prev;
/*
			if(it->first == -1 || j == -1) {
				belief += trans_prob(it->first, j, numconsiderfrms, gausstab)*postprob_prev;
			}
			else {
				belief += t_p[it->first][j]*postprob_prev;
//				printf("sim_score[%d][%d] = %.7f trans_prob between %d and %d = %.7f\n",it->first, j, sim_scores[it->first][j], it->first, j, t_p[it->first][j] );
			}
*/
//			belief += trans_prob(it->first, j, numconsiderfrms, gausstab)*postprob_prev;
//			printf("trans_prob between %d and %d = %.7f\n",it->first, j, trans_prob(it->first, j, numconsiderfrms, gausstab) );
//			trans_prob_sum += trans_prob(it->first, j, numconsiderfrms);
		}
//		printf("%d's post_prob = %.7f  \n", it->first, lhood*belief);
//		printf("%d's belief = %.7f\n", it->first, belief);
		lc_prob.insert(std::pair<int,float>(it->first, lhood*belief));
//		lc_prob.insert(std::pair<int,float>(it->first, 1.0f));
		eta += lhood*belief;
//		cout << "  eta = " << eta << endl;
		k++;
//		printf("trans_prob_sum = %.7f\n",trans_prob_sum);
	}
//	if(eta == 0.0) return;
//	eta = 1.0/eta;
//	double sum=0.0;
	for(it = lc_prob.begin(); it != lc_prob.end(); ++it) {
		if(eta != 0.0) {
			it->second = it->second/eta;
		}
		else {
			it->second = 1.0f / (float)lc_prob.size();
		}
//			sum+=it->second;
	}
//	printf("post_prob_sum = %.7f\n", sum);
}

template <int K, int L, int D>
bool voctree_t<K,L,D>::isLoopClosing(std::vector<pair<int,float> > &top_post_prob, int *lcfrmid) {
	if(doc_size < NUM_NON_CONSIDER_RECENT_FRAMES) return false;
	std::vector<pair<int,float> >::iterator it;
	std::vector<pair<int,float> >::iterator jt;
	float threshold = THRESHOLD;
	float prob_sum = 0.0f;
	float prob_max = 0.0f;
	int max_id = 0;
	static int pr_cnt;
	int seqlen = NUM_CONSIDER_SEQ_LENGTH;
//	if(lc_prob.size() < NUM_NON_CONSIDER_RECENT_FRAMES) return false;
	for(it = top_post_prob.begin(); it != top_post_prob.end()-seqlen; ++it) {
//		printf("%d's  post probability = %.7f\n", it->first, it->second);
		if(it->first == -1) continue;
		if( abs(it->first - (it+1)->first) > 1 ) continue;
		jt = it;
		prob_sum = 0.0f;
		prob_max = 0.0f;
		for(int i=0; i<seqlen+1; i++) {
			if( abs(jt->first - (jt+1)->first) > 1 && i != seqlen ) continue;
			prob_sum += jt->second;
			if(jt->second > prob_max) {
				prob_max = jt->second;
				max_id = jt->first;
			}
//			printf("jt->second = %.7f \n", jt->second);
//			printf("jt->first = %d \n", jt->first);
			jt++;
		}
//		printf("\n\n");
// cout << "prob_sum: " << prob_sum << endl;
		if(prob_sum >= threshold) {
			*lcfrmid = max_id;
			printf("     Loop closure occured with %d(%d)\n",max_id, ++pr_cnt );
			return true;
		}
	}

	return false;
}



//-----------------------------------------------------------------------------
#endif //_VOCTREE_H_

