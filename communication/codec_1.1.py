"""
this is a beta version of coder_1.0.py.
this program has a weakness that it doesn't function properly in case a digit of data changes.

"""

class Codec:

    def __init__(self, tags:dict, packet:int=64):
        """
        tags: a dict including tugs of data as key and index of a decimal point as a value
        packet: data capacity you can send once
        -----------------------------------------------
        This class has methods to encoder data to be send and decoder data you received.
        """
        self.tags = tags            # dict
        self.packet = packet        # int

    def find_tag(self, data:list):
        """
        data: list in which each value is str type
        ------------------------------------------------------------------------------
        this method search (a/) tag(/s) you have not set (a/) property(/s) of (a/) tag(/s) 
        """
        tag_notset = [i for i in data if "A" <= i <= "z"]

        for tag in tag_notset:
            self.tags[tag] = None

    def extract_data(self, data:list)-> "tag:str, data_tag:list, data_rest:list":
        """
        data: list in which each value is str type
        -----------------------------------------------------------------
        this method extracts data corresponded to a tag.
        the method separates given list to a tag, data corresponded to the tag, and rest of data.
        """

        data = data
        tag = data.pop(0)       # the head of data should be a tag
        data_tag = []           # initialize data corresponded to the tag as a list

        if tag in self.tags.keys():

            # append data to data_tag until the next tag is found 
            while not(data[0] in self.tags.keys()):

                # case in which you have not set (a/) tag(/s)
                if "A" <= data[0] <= "z":
                    self.find_tag(data)
                    break

                data_tag.append(data.pop(0))

                # all data was read
                if data == []:
                    break

            return tag, data_tag, data      # "data" means rest of data 

        else:
            print("error")
            return None, None, None

    @staticmethod
    def reshape_data(data:list, index:int)-> "return reshape data of float":
        """
        data: list of data, whose each number is str type
        index: index to insert "."
        --------------------------------------------------
        return reshaped data of float
        """
        data.insert(index, ".")
        data_reshaped = "".join(data)
        return float(data_reshaped)


    def encoder(self, flag:str, tag_data:dict)-> "encoded data:str":
        """
        flag: flag code of a str
        tag_data: a dictionary whose key is a tag and value is data
        -------------------------------------------------------------
        this method encodes given data.
        """

        data_encoded = []               # initilize as a list
        data_encoded.append(flag)       # append flag data first

        # append each tag and data transformed into str
        for key, val in tag_data.items():

            val = str(val).replace(".", "")

            # if a packet is full, then rest of data is ignored and not sent
            tag_packet = len(key) + len(val)
            if self.packet - tag_packet < len(data_encoded):
                print("caution: can't encode all data because of too much packet, but send some of it.")
                break

            data_encoded.append(key)
            data_encoded.append(val)

        return "".join(data_encoded)        # transform a list into a str

    def decoder(self, data:str)-> "flag:str, data_decoded:dict":
        """
        data: str data you recieved
        -----------------------------------------------------
        this method decodes given data.
        """
        data = list(data)
        
        flag = "".join([data.pop(0), data.pop(0)])
        data_decoded = {}

        while data != []:
            tag, data_tag, data = self.extract_data(data)

            if  self.tags[tag] == None:
                print("CAUTION: you have not completed initilization of tags")
                continue

            data_decoded[tag] = Codec.reshape_data(data_tag, self.tags[tag])           # append a tag and data

        return flag, data_decoded

    def formater(self, data_decoded:dict)-> "data_formatted":
        """
        data_decoded: dict data that is already decoded with decoder method
        -----------------------------------------------------------------------
        this method puts data into a format. 
        """
        tag_notget = [tag for tag in self.tags.keys() if not(tag in data_decoded.keys())]
        for tag in tag_notget:
            data_decoded[tag] = None

        return data_decoded


if __name__ == "__main__":
    tags = {"P": 1, "T": 2, "H": 2}
    coder = Codec(tags)
    data = {"P": 1.012, "T": 20.3}
    encoded_data = coder.encoder("21", data)
    print("encoded data:", encoded_data)

    flag, decoded_data = coder.decoder(encoded_data)

    print("flag:", flag)
    print("decoded data:", decoded_data)

    formatted_data = coder.formater(decoded_data)
    print("formatted_data:", formatted_data)

