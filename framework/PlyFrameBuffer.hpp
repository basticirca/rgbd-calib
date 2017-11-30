#ifndef PLY_FRAME_BUFFER
#define PLY_FRAME_BUFFER

#include <string>
#include <vector>
#include <map>
#include <regex>
#include <DataTypes.hpp>
#include <PlyParser.hpp>

struct Meta {
    Meta(const std::string d="./", const std::string& f = "dddd", unsigned s=0, unsigned e=1)
        : directory(d)
        , format(f)
        , start_frame(s)
        , end_frame(e)
    {}

    Meta(const Meta& m)
    	: directory(m.directory)
    	, format(m.format)
    	, start_frame(m.start_frame)
    	, end_frame(m.end_frame)
	{}

    std::string directory;
    std::string format;
    unsigned start_frame;
    unsigned end_frame;
};

class PlyFrameBuffer {
public:
    PlyFrameBuffer(const Meta& m = Meta());
    ~PlyFrameBuffer();

    void setMeta(const Meta& m);
    const Meta& getMeta() const;

    unsigned getFrameCount() const;
    PlyParser* getFrame(unsigned frame);

private:
	void initFilenames();

	std::vector<std::string> file_paths_;
    std::map<unsigned, PlyParser*> parsers_;
    Meta meta_;
};

#endif  // #ifndef PLY_FRAME_BUFFER