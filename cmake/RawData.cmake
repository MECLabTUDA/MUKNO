function(mukno_define_rawdata varname module)
  set(MUKNO_RAWDATA_SOURCE -NOTFOUND CACHE PATH "Path to the local storage of the raw data required by the following modules: ${module}")
  get_property(doc_string CACHE MUKNO_RAWDATA_SOURCE PROPERTY HELPSTRING)
  if(NOT doc_string MATCHES "[:,] ${module}")
    set_property(CACHE MUKNO_RAWDATA_SOURCE APPEND PROPERTY HELPSTRING ", ${module}")
  endif()
  if(NOT MUKNO_RAWDATA_SOURCE)
    message(FATAL_ERROR "The variable MUKNO_RAWDATA_SOURCE is required for module ${module}, but not set.")
  endif()
  set(${varname} ${MUKNO_RAWDATA_SOURCE} PARENT_SCOPE)
endfunction()