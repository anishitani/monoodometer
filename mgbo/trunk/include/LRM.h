/*
 * LRM.h
 *
 *  Created on: Mar 29, 2014
 *      Author: nishitani
 */

#ifndef LRM_H_
#define LRM_H_

#include <vector>
#include <algorithm>

#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <viso_mono.h>
#include <viso_stereo.h>

#include <ESM.h>


#define N_ODOM 3
#define CONFIGURATION_FILE "config/lrm.js"

/*
 * Configurações do programa principal
 * O arquivo de configurações está escrito em JSON
 */
#define OPTION_LOG "option.log"
#define OPTION_IMAGE "option.image"

#define PATH_SEQUENCE_BASE "path.sequence.base"
#define PATH_SEQUENCE_LEFT "path.sequence.left"
#define PATH_SEQUENCE_RIGHT "path.sequence.right"

/**
 * Os tipos path são definidos com uma base e os demais diretórios e arquivos
 * dentro da base
 * A função lê a árvore de busca (arquivo de configuração) e monta o caminho
 * base do tipo (calibarção, sequência de imagem, etc)
 *
 * @param _type Nome do tipo para o qual se busca o caminho base
 * @param pt Árvore de busca
 * @return Caminho percorrido para o tipo
 */
std::string mount_base_path(std::string _type, boost::property_tree::ptree pt)
{
	std::string _base;

	BOOST_FOREACH(const boost::property_tree::ptree::value_type &folder,
			pt.get_child("path." + _type + ".base"))
	{
		_base += "/" + folder.second.data();
	}
	return _base;
}

/**
 * Constrói o caminho para um diretório de um subtipo (pasta ou arquivo)
 *
 * @param _type	Tipo da informação buscada
 * @param _subtype Subtipo da informação buscada
 * @param pt Árvore de busca
 * @return Caminha para o subtipo
 */
std::string get_path(std::string _type, std::string _subtype,
		boost::property_tree::ptree pt)
{
	std::string _base = mount_base_path(_type, pt);
	return (_base + "/" + pt.get<std::string>("path." + _type + "." + _subtype));
}

#endif /* LRM_H_ */
