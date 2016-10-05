/*!
  \file        cached_files_map.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/12/29

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

#ifndef CACHED_FILES_MAP_H
#define CACHED_FILES_MAP_H

#include <map>
#include "utils.h"

class CachedFilesMap {
public:
  typedef std::string Key;
  typedef std::string Filename;
  CachedFilesMap(const std::string & abs_csv_filename) {
    _abs_csv_filename = abs_csv_filename;
    _cache_directory = utils::extract_folder_from_full_path(_abs_csv_filename);
    load_csv_file();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline bool has_cached_file(const Key & key) {
    Filename filename = "";
    return get_cached_file(key, filename);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline bool get_cached_file(const Key & key, Filename & abs_filename_out) {
    Filename rel_filename = "";
    Key key_clean = clean_key(key);
    if (!utils::direct_search(_map, key_clean, rel_filename))
      return false;
    abs_filename_out = relative2absolute(rel_filename);
    //! check file exists
    if (!utils::file_exists(abs_filename_out)) {
      printf("Cached file '%s' could not be accessed!\n", abs_filename_out.c_str());
      _map.erase(key);
      return false;
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  /*!
   * \brief add_cached_file
   * \param key
   * \param abs_filename_in
   * \return
   *   false if key already present
   */
  bool add_cached_file(const Key & key, const Filename & abs_filename_in) {
    Key key_clean = clean_key(key);
    // generate compatible filename, stripping all weird characters
    std::string new_filename_prefix = utils::remove_accents(key).substr(0, 50);
    unsigned int nchars = new_filename_prefix.size();
    for (unsigned int char_idx = 0; char_idx < nchars; ++char_idx) {
      char c = new_filename_prefix[char_idx];
      if ((c >= 'a' && c <='z')
          || (c >= 'A' && c <='Z')
          || (c >= '0' && c <='9'))
        continue;
      new_filename_prefix[char_idx] = '_';
    } // end for char_idx
    // add extension of abs_filename_in to new_filename
    std::string extension = utils::get_filename_extension(abs_filename_in);

    // if new_filename already exist, add suffix
    unsigned int suffix = 2;
    std::string rel_new_filename = new_filename_prefix + extension;
    while (utils::file_exists(relative2absolute(rel_new_filename))) {
      rel_new_filename = new_filename_prefix + utils::cast_to_string(suffix)
                         + extension;
      ++suffix;
    }

    // copy filename -> _cache_directory/new_filename
    // copy_file(const path& from, const path& to, BOOST_SCOPED_ENUM(copy_option) option
    // copy_option: {none, fail_if_exists = none, overwrite_if_exists};
    try {
      boost::filesystem::copy_file(abs_filename_in, relative2absolute(rel_new_filename),
                                   boost::filesystem::copy_option::fail_if_exists);
    }
    catch (boost::filesystem::filesystem_error & e) {
      printf("CachedFilesMap: boost error '%s'\n", e.what());
      return false;
    }

    // add (key, new_filename) to map
    // the second is a bool that is true if the pair was actually inserted.
    if(!_map.insert(std::pair<Key, Filename>(key_clean, rel_new_filename)).second) {
      printf("CachedFilesMap: key '%s' already present\n", key_clean.c_str());
      return false;
    }

    return save_csv_file();
  }

  //////////////////////////////////////////////////////////////////////////////

  inline unsigned int nb_cached_files() const { return _map.size(); }
  inline std::string get_cache_directory() const { return _cache_directory; }

protected:
  //////////////////////////////////////////////////////////////////////////////

  bool load_csv_file(bool clear_before = true) {
    if (clear_before)
      _map.clear();
    // create folder if needed
    if (!utils::file_exists(_abs_csv_filename)) {
      if (!utils::directory_exists(_cache_directory)) {
        printf("CachedFilesMap: creating new folder '%s'\n", _cache_directory.c_str());
        return utils::create_directory(_cache_directory);
      }
      printf("CachedFilesMap: could not open file '%s'\n", _abs_csv_filename.c_str());
      return false;
    }

    std::vector<std::string> lines;
    if (!utils::retrieve_file_split(_abs_csv_filename, lines, true, true)) {
      printf("CachedFilesMap: could not open file '%s'\n", _abs_csv_filename.c_str());
      return false;
    }
    // split lines
    unsigned int nlines = lines.size();
    for (unsigned int line_idx = 0; line_idx < nlines; ++line_idx) {
      std::string line = lines[line_idx];
      std::string::size_type comma_pos = line.find(',');
      if (comma_pos == std::string::npos) {
        printf("CachedFilesMap: line %i does not have a comma:'%s'\n",
               line_idx, line.c_str());
        continue;
      }
      Key key = line.substr(0, comma_pos);
      Filename rel_filename = line.substr(comma_pos+1),
          abs_filename = relative2absolute(rel_filename);
      if(!utils::file_exists(abs_filename)) {
        printf("CachedFilesMap: non existing filename '%s' in entry!\n", abs_filename.c_str());
        continue;
      }
      _map.insert(std::pair<Key, Filename>(key, rel_filename));
    } // end for line_idx
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool save_csv_file() {
    //load_csv_file(false); // reload file for good measure (if it was changed meanwhile)
    std::ostringstream out;
    std::map<Key, Filename>::const_iterator it = _map.begin();
    while (it != _map.end()) {
      out << it->first << "," << it->second << std::endl;
      ++it;
    }
    return utils::save_file(_abs_csv_filename, out.str());
  }

protected:

  //! remove commas from key
  inline Key clean_key(const Key & key) const {
    Key key_clean = key;
    utils::find_and_replace(key_clean, ",", "");
    utils::find_and_replace(key_clean, "\n", " ");
    return key_clean;
  }


  //////////////////////////////////////////////////////////////////////////////

  inline std::string relative2absolute(const Filename & rel_filename_in) const {
    std::ostringstream out;
    out << _cache_directory << rel_filename_in;
    return out.str();
  }

  //////////////////////////////////////////////////////////////////////////////

  std::map<Key, Filename> _map;
  std::string _cache_directory, _abs_csv_filename;
}; // end CachedFilesMap

#endif // CACHED_FILES_MAP_H
