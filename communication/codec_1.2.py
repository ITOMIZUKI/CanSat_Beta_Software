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

        self.tags[tag] <- index of decimal point
        """
        self.tags = tags            # dict
        self.packet = packet        # int
        self.decimal = {}           # dict
        
        # init all values of self.decimal as False
        # this values are changed when a decimal point is detected in encoded data
        # if a value is change into True, it's going to be re-changed into Flase after decode
        for tag in self.tags.keys():
            self.decimal[tag] = False

    def find_tag(self, data:list):
        """
        data: list in which each value is str type
        ------------------------------------------------------------------------------
        this method search (a/) tag(/s) you have not set (a/) property(/s) of (a/) tag(/s) 
        """
        tag_notset = [i for i in data if "A" <= i <= "z"]

        for tag in tag_notset:
            self.tags.append(tag)

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

                # case in which you have not set a tag
                if "A" <= data[0] <= "z":
                    self.find_tag(data)
                    break

                # case in which head of data has a decimal point
                elif data[0] == ".":
                    self.decimal[tag] = True

                data_tag.append(data.pop(0))

                # all data was read
                if data == []:
                    break

            return tag, data_tag, data      # "data" means rest of data 

        else:
            print("error")
            return None, None, None

    def reshape_data(self, data:list, tag:str)-> "return reshape data of float":
        """
        data: list of data, whose each number is str type
        tag: a tag corresponded to given data
        --------------------------------------------------
        return reshaped data of float
        """

        # check each value of self.decimal
        if self.decimal[tag]:
            self.decimal[tag] = False               # reset
        else:
            data.insert(self.tags[tag], ".")        # insert a decimal point at a default point

        data_reshaped = "".join(data)               # list --> str
        return float(data_reshaped)                 # str  --> float


    def encoder(self, flag:str, tag_data:dict, tag_decimal:tuple=tuple())-> "encoded data:str":
        """
        flag: flag code of a str
        tag_data: a dictionary whose key is a tag and value is data
        tag_decimal=(,): a tuple including tugs, whose decimal points are not removed  
        -------------------------------------------------------------
        this method encodes given data.
        """

        data_encoded = []               # initilize as a list
        data_encoded.append(flag)       # append flag data first

        # append each tag and data transformed into str
        for key, val in tag_data.items():

            # comfirm whether decimal points should be removed or not
            if key in tag_decimal:
                val = str(val)
            else:
                val = str(val).replace(".", "")

            # if a packet is full, then rest of data is ignored and not sent
            tag_packet = len(key) + len(val)
            if self.packet - tag_packet < len(data_encoded):
                print("CAUTION: can't encode all data because of too much packet, but send some of it.")
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
            data_decoded[tag] = self.reshape_data(data_tag, tag)           # append a tag and data

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

    # init
    tags = {"P": 1, "T": 2, "H": 2, "A": 1}
    coder = Codec(tags)

    # send in default setting
    data = {"P": 1.012, "T": 20.3, "H": 20.8, "A": 9.8}
    encoded_data = coder.encoder("21", data)                # encode without setting
    print("raw data:", data)
    print("encoded data:", encoded_data)

    flag, decoded_data = coder.decoder(encoded_data)        # decode
    print("flag:", flag)
    print("decoded data:", decoded_data)
    print()

    # send, not removing the decimal point of "A"
    data = {"P": 1.013, "T": 20.6, "H": 20.4, "A": 23.9}
    encoded_data = coder.encoder("30", data, tag_decimal=("A",))    # encode, specifying tag_decimal argument
    print("raw data:", data)
    print("encoded data:", encoded_data)

    flag, decoded_data = coder.decoder(encoded_data)                # decode
    print("flag:", flag)
    print("decoded data:", decoded_data)