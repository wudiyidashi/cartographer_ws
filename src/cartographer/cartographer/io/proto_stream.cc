/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/io/proto_stream.h"

#include "glog/logging.h"

namespace cartographer {
namespace io {

namespace {

// First eight bytes to identify our proto stream format.
const uint64 kMagic = 0x7b1d1f7b5bf501db;

// д��8���ֽڵ�У��λ
void WriteSizeAsLittleEndian(uint64 size, std::ostream* out) {
  for (int i = 0; i != 8; ++i) {
    out->put(size & 0xff);
    size >>= 8;
  }
}

// ��ȡǰ8���ֽڵ�ֵ, �����ۼ�
bool ReadSizeAsLittleEndian(std::istream* in, uint64* size) {
  *size = 0;
  for (int i = 0; i != 8; ++i) {
    *size >>= 8;
    *size += static_cast<uint64>(in->get()) << 56;
  }
  return !in->fail();
}

}  // namespace

// �Զ����Ʒ�ʽ, д��ķ�ʽ���ļ�, ��д��8���ֽڵ�����У��
ProtoStreamWriter::ProtoStreamWriter(const std::string& filename)
    : out_(filename, std::ios::out | std::ios::binary) {
  WriteSizeAsLittleEndian(kMagic, &out_);
}

// ������������Ƚ���ѹ��, ��д�뵽�ļ���
void ProtoStreamWriter::Write(const std::string& uncompressed_data) {
  std::string compressed_data;
  // �����ݽ���ѹ��
  common::FastGzipString(uncompressed_data, &compressed_data);
  // �������ݵ�sizeд���ļ�
  WriteSizeAsLittleEndian(compressed_data.size(), &out_);
  // ���ڴ��� compressed_data �Զ����Ƶ���ʽд���ļ�
  out_.write(compressed_data.data(), compressed_data.size());
}

// ������д���ļ���
void ProtoStreamWriter::WriteProto(const google::protobuf::Message& proto) {
  std::string uncompressed_data;
  proto.SerializeToString(&uncompressed_data);
  // ѹ����д��
  Write(uncompressed_data);
}

// �رմ򿪵��ļ�
bool ProtoStreamWriter::Close() {
  out_.close();
  return !out_.fail();
}


// ��ȡpbstream�ļ�, ����ǰ8���ֽڵ����ݽ���У��
ProtoStreamReader::ProtoStreamReader(const std::string& filename)
    : in_(filename, std::ios::in | std::ios::binary) {
  uint64 magic;
  // ��ǰ8���ֽڵ����ݽ���У��
  if (!ReadSizeAsLittleEndian(&in_, &magic) || magic != kMagic) {
    in_.setstate(std::ios::failbit);
  }
  CHECK(in_.good()) << "Failed to open proto stream '" << filename << "'.";
}

// ��ȡ���ݲ���ѹ
bool ProtoStreamReader::Read(std::string* decompressed_data) {
  uint64 compressed_size;
  // ��ȡ���ݵ�size
  if (!ReadSizeAsLittleEndian(&in_, &compressed_size)) {
    return false;
  }
  // ����size�����ַ���
  std::string compressed_data(compressed_size, '\0');
  // ��ȡ���ݷ���compressed_data��
  if (!in_.read(&compressed_data.front(), compressed_size)) {
    return false;
  }
  // ���н�ѹ
  common::FastGunzipString(compressed_data, decompressed_data);
  return true;
}

// ��ȡ���ݲ�����protobuf��ʽ������
bool ProtoStreamReader::ReadProto(google::protobuf::Message* proto) {
  std::string decompressed_data;
  return Read(&decompressed_data) && proto->ParseFromString(decompressed_data);
}

bool ProtoStreamReader::eof() const { return in_.eof(); }

}  // namespace io
}  // namespace cartographer
