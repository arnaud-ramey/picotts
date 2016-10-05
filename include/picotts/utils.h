/*!
  \file        utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2015/11/7

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file
*/
#ifndef UTILS_H
#define UTILS_H

#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>

namespace utils {
typedef std::string Language;

/*! get the content of a file on the hard drive
 * \param filepath for instance /foo/bar/test.txt
 * \param ans where we save the answer
 * \return true if success, false otherwise
 */
inline bool retrieve_file(const std::string & filepath, std::string & ans) {
  // printf("retrieve_file('%s')", filepath.c_str());
  // open the file
  std::ifstream myfile(filepath.c_str(), std::ios::in);
  /*
   * check if success
   */
  if (!myfile || myfile.is_open() == false) {
    // error while reading the file
    printf("Unable to open file '%s'\n", filepath.c_str());
    return false;
  }
  /*
   * concatenate to buffer
   */
  std::string line;
  std::ostringstream buffer;
  bool first_line = true;
  while (myfile.good()) {
    // add a cariage return if it is not the first line
    if (first_line)
      first_line = false;
    else
      buffer << std::endl;

    getline(myfile, line);
    buffer << line;
  } // end myfine.good()
  myfile.close();
  ans = buffer.str();
  return true;
} // end retrieve_file()

////////////////////////////////////////////////////////////////////////////////

/*! get the content of a file on the hard drive
 * \param filepath for instance /foo/bar/test.txt
 * \param ans where we save the answer - one line in the vector for each file line
 */
inline bool retrieve_file_split(const std::string & filepath,
                                std::vector<std::string> & ans,
                                bool remove_empty_lines = false,
                                bool remove_empty_last_line = true) {
  //printf("retrieve_file_split('%s')", filepath.c_str());
  // open the file
  std::ifstream myfile(filepath.c_str(), std::ios::in);
  if (!myfile.is_open()) {// error while reading the file
    printf("Unable to open file '%s'\n", filepath.c_str());
    return false;
  }
  std::string line;
  ans.clear();
  while (myfile.good()) {
    getline(myfile, line);
    if (remove_empty_lines && line.empty())
      continue;
    ans.push_back(line);
  } // end myfine.good()
  if (remove_empty_last_line && ans.back().empty())
    ans.pop_back();
  myfile.close();
  return true;
}

////////////////////////////////////////////////////////////////////////////////

/*! Save a string to a given file
 * \param filepath the filename where to save the content,
    for instance "/tmp/foo.txt"
 * \param content the string to save
 */
inline bool save_file(const std::string & filepath, const std::string & content) {
  // maggieDebug2("save_file('%s')", filepath.c_str());
  std::ofstream myfile(filepath.c_str());
  if (!myfile.is_open()) { // check if success
    printf("Unable to open file '%s' for writing.\n", filepath.c_str());
    return false;
  }
  myfile << content;
  return true;
}

////////////////////////////////////////////////////////////////////////////////

//! execute a system instruction in a safe mode
inline int exec_system(const std::string & instr) {
  int return_value = system(instr.c_str());
  if (return_value != 0)
    printf("system('%s') returned %i != 0!\n", instr.c_str(), return_value);
  return return_value;
} // end system()

////////////////////////////////////////////////////////////////////////////////

/*! \brief   split a std::string
 * \param   str the long std::string
 * \param   delim the std::string which is the separator between words, for instance '/'
 * \param   results a pointer towards a vector of std::string, will contain the results
 */
inline void stringSplit(const std::string & str, const std::string & delim,
                        std::vector<std::string>* results) {
  results->clear();
  if (str == "")
    return;
  size_t delim_pos, search_pos = 0;
  while (search_pos <= str.size() - 1) {
    delim_pos = str.find(delim, search_pos);
    if (delim_pos == std::string::npos) { // no more delim
      results->push_back(str.substr(search_pos));
      return;
    }
    if (delim_pos > 0) // == 0 only happens if str starts with delim
      results->push_back(str.substr(search_pos, delim_pos - search_pos));
    search_pos = delim_pos + delim.size();
    // quit if we reached the end of the std::string or std::string empty
  }
} // end std::stringSplit();

////////////////////////////////////////////////////////////////////////////////

/*! find all the iterations of a pattern in a string and replace
 * them with another pattern
 * \param stringToReplace the string to change
 * \param pattern what we want to replace
 * \param patternReplacement what we replace with
 * \return the number of times we replaced pattern
 */
inline int find_and_replace(std::string& stringToReplace,
                            const std::string & pattern,
                            const std::string & patternReplacement) {
  size_t j = 0;
  int nb_found_times = 0;
  for (; (j = stringToReplace.find(pattern, j)) != std::string::npos;) {
    //cout << "found " << pattern << endl;
    stringToReplace.replace(j, pattern.length(), patternReplacement);
    j += patternReplacement.length();
    ++ nb_found_times;
  }
  return nb_found_times;
}

////////////////////////////////////////////////////////////////////////////////

/*! cast a string to another type
 * \param in the string to cast
 * \return the cast
 */
template<class _T>
_T cast_from_string(const std::string & in) {
  std::istringstream myStream(in);
  _T ans;
  myStream >> ans;
  return ans;
}

/*! cast any type to string
 * \param in the value to cast
 * \param success true if we managed, false otherwise
 * \return the cast
 */
template<class _T>
std::string cast_to_string(const _T in) {
  std::ostringstream ans;
  ans << in;
  return ans.str();
}


////////////////////////////////////////////////////////////////////////////////

/*! remove the spaces at the end of a word
 * \param content
 */
inline void remove_trailing_spaces(std::string & content) {
  while (content.length() > 0 && (content[content.length() - 1] == ' '
                                  || content[content.length() - 1] == '\n'
                                  || content[content.length() - 1] == '\t'))
    content = content.substr(0, content.length() - 1);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * remove the spaces at the beginning of a word
 * \param content
 */
inline void remove_beginning_spaces(std::string & content) {
  while (content.length() > 0 && (content[0] == ' ' || content[0] == '\n'
                                  || content[0] == '\t'))
    content = content.substr(1);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param filename
    a relative or absolute filename
 \return true if the file with given filename exists
*/
inline bool file_exists(const std::string & filename) {
  return boost::filesystem::exists(filename)
      && boost::filesystem::is_regular_file(filename);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param directoryname
    a relative or absolute directoryname
 \see http://www.boost.org/doc/libs/1_44_0/libs/filesystem/v3/doc/reference.html
 \return true if the directory with given directoryname exists
*/
inline bool directory_exists(const std::string & directoryname) {
  return boost::filesystem::exists(directoryname)
      && boost::filesystem::is_directory(directoryname);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param directoryname
    a relative or absolute directoryname
 \see http://www.boost.org/doc/libs/1_44_0/libs/filesystem/v3/doc/reference.html
 \return true if a new directory was created, otherwise false
*/
inline bool create_directory(const std::string & directoryname) {
  return boost::filesystem::create_directory(directoryname);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param path
 \return std::string
 \example path="/tmp/foo/bar.dat", returns "/tmp/foo/"
 \example path="foo/bar.dat", returns "foo/"
 \example path="/bar.dat", returns "/"
 \example path="bar.dat", returns ""
*/
inline std::string extract_folder_from_full_path(const std::string & path) {
  std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos == std::string::npos)
    return "";
  return path.substr(0, 1 + slash_pos);
} // end extract_folder_from_full_path()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Retrieve the extension from a filename
 * \param path
 *  the full path
 * \example
 *  "/foo/bar" -> ""
 *  "/foo/bar.dat" -> ".dat"
 *  "/foo.zim/bar.dat" -> ".dat"
 *  "/foo.zim/bar" -> ""
 */
inline std::string get_filename_extension
(const std::string & path) {
  std::string::size_type dot_pos = path.find_last_of('.');
  if (dot_pos == std::string::npos)
    return "";
  std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos != std::string::npos && slash_pos > dot_pos) // dot before slash
    return "";
  return path.substr(dot_pos);
}

////////////////////////////////////////////////////////////////////////////////

/*! remove all the spanish characters in a string, that is
 * ¿ -> "", ¡ -> "",
 * à -> a,  á -> a,  é -> e, í -> i,
 * ó -> o,  ú -> u,  ü -> u, ñ -> ny
 * \param string the string to clean
 */
inline void clean_spanish_chars(std::string& string) {
  find_and_replace(string, "¿", "");
  find_and_replace(string, "¡", "");
  find_and_replace(string, "á", "a");
  find_and_replace(string, "Á", "A");
  find_and_replace(string, "ç", "c");
  find_and_replace(string, "é", "e");
  find_and_replace(string, "É", "E");
  find_and_replace(string, "í", "i");
  find_and_replace(string, "Í", "I");
  find_and_replace(string, "ó", "o");
  find_and_replace(string, "Ó", "O");
  find_and_replace(string, "ú", "u");
  find_and_replace(string, "Ú", "U");
  find_and_replace(string, "ñ", "ny");
  find_and_replace(string, "Ñ", "NY");
}

////////////////////////////////////////////////////////////////////////////////

/*! change all accented letters to normal letters
 ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×ØÙÚÛÜÝÞßàáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ
 \example eéèêaàäâçc would become eeeeaaaacc */
inline std::string remove_accents(const std::string & str) {
  std::string ans = str;
  clean_spanish_chars(ans);
  find_and_replace(ans, "â", "a");
  find_and_replace(ans, "Â", "A");
  find_and_replace(ans, "à", "a");
  find_and_replace(ans, "À", "A");
  find_and_replace(ans, "è", "e");
  find_and_replace(ans, "È", "E");
  find_and_replace(ans, "ê", "e");
  find_and_replace(ans, "Ê", "E");
  find_and_replace(ans, "î", "i");
  find_and_replace(ans, "Î", "I");
  find_and_replace(ans, "ô", "o");
  find_and_replace(ans, "Ô", "O");
  find_and_replace(ans, "ù", "u");
  find_and_replace(ans, "Ù", "U");
  find_and_replace(ans, "û", "u");
  find_and_replace(ans, "Û", "U");
  find_and_replace(ans, "ü", "u");
  find_and_replace(ans, "Ü", "U");
  return ans;
}


////////////////////////////////////////////////////////////////////////////////

std::string extract_from_tags(const std::string & content,
                              const std::string & block_begin,
                              const std::string & block_end,
                              int & initial_search_pos) {
  // find the beginning
  size_t pos_begin = content.find(block_begin, initial_search_pos);
  if (pos_begin == std::string::npos) {
    ROS_DEBUG("block_begin '%s' could not be found", block_begin.c_str());
    return "";
  }
  // remove the block at the beginning
  pos_begin += block_begin.length();
  // find the end
  size_t pos_end = content.find(block_end, pos_begin);
  if (pos_end == std::string::npos) {
    ROS_DEBUG("block_end '%s' could not be found", block_end.c_str());
    return "";
  }
  // update the index
  initial_search_pos = pos_end + block_end.length();
  // extract the substring
  std::string ans = content.substr(pos_begin, pos_end - pos_begin);
  return ans;
}

////////////////////////////////////////////////////////////////////////////////

inline bool extract_language(const std::string & sentence,
                             Language & ans) {
  size_t sep_pos = sentence.find(':');
  if ( sep_pos == std::string::npos )
    return false;
  ans = sentence.substr(0, sep_pos);
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool find_given_language_in_multilanguage_line(
    const std::vector<std::string> & versions,
    const Language target_language,
    std::string & ans) {
  // check in each version if it was found
  Language current_language;
  std::vector<std::string> good_versions;
  for(std::vector<std::string>::const_iterator current_version = versions.begin();
      current_version != versions.end() ; ++current_version) {
    if (!extract_language(*current_version, current_language)
        || current_language != target_language)
      continue;
    // we remove the prefix and the colon
    good_versions.push_back(current_version->substr(1 + target_language.size()));
  } // end loop versions
  if (good_versions.empty())
    return false;
  ans = good_versions.at(rand() % good_versions.size());
  return true;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * Perform a direct search on a map.
 * \param map
 *   The map to be searched.
 * \param search_key
 *   The key we want.
 * \param value_lookup_result (out)
 *   The value corresponding to \a search_key in the map,
 *   if \a search_key is present.
 *   Undetermined otherwise.
 * \return
 *  true if \a search_key was found.
 * \example map=[1->"value1"; 2->"value2"; 3->"value3"; 4->"value4"]
 *  search_key=0: returns false, value_lookup_result not affected
 *  search_key=1: returns true, value_lookup_result="value1"
 *  search_key=2: returns true, value_lookup_result="value2"
 */
template<class _Key, class _Value>
inline bool direct_search(const std::map<_Key, _Value> & map,
                           const _Key & search_key,
                           _Value & value_lookup_result) {
  typename std::map<_Key, _Value>::const_iterator it = map.find(search_key);
  if (it == map.end()) {
    return false;
  }
  value_lookup_result = it->second;
  return true;
} // end direct_search()

////////////////////////////////////////////////////////////////////////////////

/*!
 * Perform an inverse search on a map.
 * \param map
 *   The map to be searched.
 * \param search_value
 *   The value we want.
 * \param key_lookup_result (out)
 *   The key corresponding to \a search_value in the map,
 *   if \a search_value is present.
 *   Undetermined otherwise.
 * \return
 *  true if \a search_value was found.
 * \example map=[1->"value1"; 2->"value2"; 3->"value3"; 4->"value4"]
 *  search_value="value0": returns false, key_lookup_result not affected
 *  search_value="value1": returns true, key_lookup_result=1
 *  search_value="value2": returns true, key_lookup_result=2
 */
template<class _Key, class _Value>
inline bool reverse_search (const std::map<_Key, _Value> & map,
                            const _Value & search_value,
                            _Key & key_lookup_result) {
  for (typename std::map<_Key, _Value>::const_iterator map_iterator = map.begin();
       map_iterator != map.end();
       map_iterator ++) {
    if (map_iterator->second == search_value) {
      key_lookup_result = map_iterator->first;
      return true;
    }
  } // end loop map_iterator
  return false;
} // end reverse_search()

} // end namespace utils

#endif // UTILS_H

