# Metadata describing one (physical) object model.
string                            model_id         # UUID4; also the cache entry name in download_path
string                            model_type       # one of: 'sdf', 'stl', 'dae', 'obj'
ObjectModelStorageMappingValue[]  filter_keys      # tags used by get_models_info filtering
string                            metadata_json    # arbitrary JSON metadata; empty when absent
